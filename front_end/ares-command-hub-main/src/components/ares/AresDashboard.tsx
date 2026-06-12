import { useEffect, useMemo, useState } from "react";
import type { ComponentType } from "react";
import {
  Activity,
  AlertTriangle,
  Cctv,
  CircleDot,
  Compass,
  Cpu,
  Database,
  Gauge,
  ImageIcon,
  MapPin,
  Network,
  Power,
  Radar,
  Radio,
  Wifi,
  WifiOff,
} from "lucide-react";
import {
  ResponsiveContainer,
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
} from "recharts";

type NodeStatus = "online" | "offline" | "degraded";
type ConnectionStatus = "connected" | "degraded" | "offline";

type Telemetry = {
  camera_stream_url: string;
  camera_status: NodeStatus;
  linear_velocity: number;
  angular_velocity: number;
  odom_x: number;
  odom_y: number;
  odom_yaw: number;
  odometry_status: NodeStatus;
  velocity_status: NodeStatus;
  connection_status: ConnectionStatus;
  last_update: string;
};

type RosTopic = {
  name: string;
  types: string[];
  subscribed: boolean;
  status: string;
  channel?: string | null;
  last_seen?: number | null;
};

type RosTopicsResponse = {
  ros_available: boolean;
  error?: string | null;
  topics: RosTopic[];
};

type RtabmapStatus = {
  rtabmap_online: boolean;
  visual_odometry: boolean;
  icp_odometry: boolean;
  tracking_status: string;
  frame_id: string;
  rgb_topic: string;
  imu_topic: string;
  camera_info_topic: string;
  sources: Record<string, string>;
};

type WsPacket<TData> = {
  type: string;
  topic: string;
  stamp: number | null;
  data: TData | null;
  status: string;
  channel?: string;
  error?: string | null;
  last_seen_age?: number | null;
  stale?: boolean;
};

type ImageFrame = {
  data?: string;
  width?: number;
  height?: number;
  encoding?: string;
  format?: string;
  header?: {
    frame_id?: string;
    stamp?: number | null;
  };
};

const API_BASE = import.meta.env.VITE_API_BASE_URL ?? "http://127.0.0.1:8000";
const WS_BASE = API_BASE.replace(/^http/, "ws");

const DEFAULT_TELEMETRY: Telemetry = {
  camera_stream_url: "",
  camera_status: "offline",
  linear_velocity: 0,
  angular_velocity: 0,
  odom_x: 0,
  odom_y: 0,
  odom_yaw: 0,
  odometry_status: "offline",
  velocity_status: "offline",
  connection_status: "offline",
  last_update: "--:--:--",
};

const DEFAULT_TOPICS: RosTopicsResponse = {
  ros_available: false,
  topics: [],
};

const DEFAULT_RTABMAP: RtabmapStatus = {
  rtabmap_online: false,
  visual_odometry: false,
  icp_odometry: false,
  tracking_status: "offline",
  frame_id: "map",
  rgb_topic: "/k4a/rgb/image_raw/compressed",
  imu_topic: "/k4a/imu_filtered",
  camera_info_topic: "/k4a/rgb/camera_info",
  sources: {},
};

const PRIMARY_TOPIC_ORDER = [
  "/k4a/rgb/image_raw/compressed",
  "/k4a/rgb/camera_info",
  "/rtabmap/odom",
  "/k4a/imu_filtered",
  "/all_available_paths",
  "/scan",
  "/goal_pose",
  "/initialpose",
];

const K4A_RGB_RESOLUTION = "2048x1536";

const EMPTY_PACKET: WsPacket<null> = {
  type: "unknown",
  topic: "",
  stamp: null,
  data: null,
  status: "offline",
};

function toBadgeStatus(status: unknown): NodeStatus | "connected" {
  if (status === "connected") return "connected";
  if (status === "online" || status === "ok" || status === true) return "online";
  if (status === "degraded" || status === "warn") return "degraded";
  return "offline";
}

function formatTopicType(types: string[] | undefined) {
  const type = types?.[0];
  if (!type) return "unknown";
  return type.split("/").slice(-2).join("/");
}

function formatAge(seconds: number | null | undefined) {
  if (seconds == null || !Number.isFinite(seconds)) return "sin datos";
  if (seconds < 1) return "<1s";
  return `${seconds.toFixed(seconds < 10 ? 1 : 0)}s`;
}

function cameraResolutionLabel(
  frame: ImageFrame | null | undefined,
  topic: string,
  fallbackUrl: string,
) {
  if (frame?.width && frame?.height) {
    return `${frame.width}x${frame.height}`;
  }
  if (frame?.data && topic === "/k4a/rgb/image_raw/compressed") {
    return K4A_RGB_RESOLUTION;
  }
  if (fallbackUrl.endsWith(".mp4")) {
    return "--";
  }
  return "--";
}

function StatusBadge({ status }: { status: NodeStatus | "connected" | "offline" | "degraded" }) {
  const map = {
    online: { dot: "on", label: "Online" },
    connected: { dot: "on", label: "Conectado" },
    degraded: { dot: "warn", label: "Degradado" },
    offline: { dot: "off", label: "Offline" },
  } as const;
  const s = map[status as keyof typeof map] ?? map.offline;
  return (
    <span className="inline-flex items-center gap-2 text-[0.72rem] uppercase tracking-widest text-muted-foreground">
      <span className={`status-dot ${s.dot} ${s.dot !== "on" ? "pulse" : ""}`} />
      {s.label}
    </span>
  );
}

function MetricCard({
  icon: Icon,
  label,
  value,
  unit,
  tone = "gold",
}: {
  icon: ComponentType<{ className?: string }>;
  label: string;
  value: string | number;
  unit?: string;
  tone?: "gold" | "steel" | "ivory";
}) {
  const toneClass =
    tone === "gold"
      ? "from-mustard/90 to-mustard/30"
      : tone === "steel"
        ? "from-graphite/90 to-graphite/30"
        : "from-ivory/90 to-ivory/30";
  return (
    <div className="panel panel-accent p-5">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2 text-muted-foreground text-xs uppercase tracking-[0.18em]">
          <Icon className="h-4 w-4 text-mustard" />
          {label}
        </div>
        <div className={`h-1.5 w-10 rounded-full bg-gradient-to-r ${toneClass}`} />
      </div>
      <div className="mt-3 flex items-baseline gap-2">
        <span className="font-display text-4xl text-ivory">{value}</span>
        {unit && <span className="text-sm text-muted-foreground">{unit}</span>}
      </div>
    </div>
  );
}

function NodeRow({
  name,
  status,
  icon: Icon,
  detail = "/ros2/node",
}: {
  name: string;
  status: NodeStatus;
  icon: ComponentType<{ className?: string }>;
  detail?: string;
}) {
  return (
    <div className="flex items-center justify-between py-3 border-b border-border last:border-0">
      <div className="flex items-center gap-3 min-w-0">
        <div className="h-9 w-9 rounded-md bg-cocoa flex items-center justify-center border border-border">
          <Icon className="h-4 w-4 text-mustard" />
        </div>
        <div className="min-w-0">
          <div className="font-medium text-ivory">{name}</div>
          <div className="text-xs text-muted-foreground truncate">{detail}</div>
        </div>
      </div>
      <StatusBadge status={status} />
    </div>
  );
}

function InfoStat({
  label,
  value,
  icon: Icon,
}: {
  label: string;
  value: string | number;
  icon: ComponentType<{ className?: string }>;
}) {
  return (
    <div className="panel p-3 min-w-0">
      <div className="flex items-center gap-2 text-muted-foreground uppercase tracking-widest text-[0.65rem]">
        <Icon className="h-3.5 w-3.5 text-mustard" />
        {label}
      </div>
      <div className="font-display text-lg text-ivory truncate">{value}</div>
    </div>
  );
}

function TopicRow({ topic }: { topic: RosTopic }) {
  return (
    <div className="grid grid-cols-[1fr_auto] gap-3 py-2 border-b border-border last:border-0">
      <div className="min-w-0">
        <div className="text-sm text-ivory truncate">{topic.name}</div>
        <div className="text-xs text-muted-foreground truncate">
          {topic.channel ?? "topic"} · {formatTopicType(topic.types)}
        </div>
      </div>
      <div className="text-right">
        <StatusBadge status={toBadgeStatus(topic.status)} />
        <div className="text-[0.65rem] uppercase tracking-widest text-muted-foreground">
          {topic.subscribed ? "sub" : "seen"}
        </div>
      </div>
    </div>
  );
}

function usePolledEndpoint<T>(path: string, fallback: T, intervalMs: number) {
  const [data, setData] = useState<T>(fallback);
  const [online, setOnline] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    const fetchData = async () => {
      try {
        const res = await fetch(`${API_BASE}${path}`);
        if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
        const json = (await res.json()) as T;
        if (!cancelled) {
          setData(json);
          setOnline(true);
          setError(null);
        }
      } catch (exc) {
        if (!cancelled) {
          setData(fallback);
          setOnline(false);
          setError(exc instanceof Error ? exc.message : String(exc));
        }
      }
    };

    fetchData();
    const id = window.setInterval(fetchData, intervalMs);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [fallback, intervalMs, path]);

  return { data, online, error };
}

function useWebSocketPacket<TData>(path: string, fallbackData: TData | null = null) {
  const [packet, setPacket] = useState<WsPacket<TData>>({
    ...EMPTY_PACKET,
    data: fallbackData,
  });
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    let closedByHook = false;
    let socket: WebSocket | null = null;
    let retryId: number | undefined;

    const connect = () => {
      socket = new WebSocket(`${WS_BASE}${path}`);

      socket.onopen = () => {
        setConnected(true);
      };

      socket.onmessage = (event) => {
        try {
          setPacket(JSON.parse(event.data) as WsPacket<TData>);
        } catch (exc) {
          setPacket({
            ...EMPTY_PACKET,
            data: fallbackData,
            status: "degraded",
            error: exc instanceof Error ? exc.message : String(exc),
          });
        }
      };

      socket.onerror = () => {
        setConnected(false);
      };

      socket.onclose = () => {
        setConnected(false);
        if (!closedByHook) retryId = window.setTimeout(connect, 2000);
      };
    };

    connect();
    return () => {
      closedByHook = true;
      if (retryId) window.clearTimeout(retryId);
      socket?.close();
    };
  }, [fallbackData, path]);

  return { packet, connected };
}

function useVelocityHistory(telemetry: Telemetry) {
  const [data, setData] = useState(() =>
    Array.from({ length: 20 }, (_, i) => ({
      t: i,
      v: Number(telemetry.linear_velocity.toFixed(3)),
      stamp: telemetry.last_update,
    })),
  );
  useEffect(() => {
    setData((prev) => {
      const last = prev[prev.length - 1];
      const next = {
        t: prev.length - 1,
        v: Number(telemetry.linear_velocity.toFixed(3)),
        stamp: telemetry.last_update,
      };
      if (last.stamp === next.stamp && last.v === next.v) {
        return prev;
      }
      return [...prev.slice(1), next].map((point, index) => ({
        ...point,
        t: index,
      }));
    });
  }, [telemetry.last_update, telemetry.linear_velocity]);
  return data;
}

export default function AresDashboard() {
  const telemetryState = usePolledEndpoint<Telemetry>("/api/telemetry", DEFAULT_TELEMETRY, 1000);
  const topicsState = usePolledEndpoint<RosTopicsResponse>("/ros/topics", DEFAULT_TOPICS, 2500);
  const rtabmapState = usePolledEndpoint<RtabmapStatus>(
    "/api/rtabmap/status",
    DEFAULT_RTABMAP,
    1500,
  );
  const cameraSocket = useWebSocketPacket<ImageFrame>("/ws/camera");
  const [estopArmed, setEstopArmed] = useState(false);
  const [estopBusy, setEstopBusy] = useState(false);
  const [estopError, setEstopError] = useState<string | null>(null);

  const telemetry = telemetryState.data;
  const topics = topicsState.data;
  const rtabmap = rtabmapState.data;
  const cameraFrame = cameraSocket.packet.data;

  const cameraStatus =
    cameraSocket.connected && !cameraSocket.packet.stale
      ? toBadgeStatus(cameraSocket.packet.status)
      : "offline";
  const cameraResolution = cameraResolutionLabel(
    cameraFrame,
    cameraSocket.packet.topic,
    telemetry.camera_stream_url,
  );
  const velData = useVelocityHistory(telemetry);

  const highlightedTopics = useMemo(() => {
    const byName = new globalThis.Map(topics.topics.map((topic) => [topic.name, topic]));
    const priority = PRIMARY_TOPIC_ORDER.map((name) => byName.get(name)).filter(
      (topic): topic is RosTopic => Boolean(topic),
    );
    return (priority.length > 0 ? priority : topics.topics).slice(0, 12);
  }, [topics.topics]);

  const handleEstopClick = async () => {
    const nextActive = !estopArmed;
    setEstopBusy(true);
    setEstopError(null);
    try {
      const res = await fetch(`${API_BASE}/api/estop`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ active: nextActive, source: "gui" }),
      });
      const data = await res.json();
      if (!res.ok || !data.ok) {
        throw new Error(data.detail ?? data.message ?? "E-STOP command failed");
      }
      setEstopArmed(nextActive);
    } catch (exc) {
      setEstopError(exc instanceof Error ? exc.message : String(exc));
    } finally {
      setEstopBusy(false);
    }
  };

  const alerts = useMemo(() => {
    const a: { msg: string; tone: "warn" | "off" }[] = [];
    if (!telemetryState.online) a.push({ msg: "Backend FastAPI sin respuesta", tone: "off" });
    if (!topics.ros_available) a.push({ msg: "ROS2 no disponible para el bridge", tone: "off" });
    if (telemetry.camera_status !== "online")
      a.push({ msg: "Nodo de cámara sin transmisión", tone: "off" });
    if (telemetry.velocity_status !== "online")
      a.push({ msg: "Nodo de velocidad inestable", tone: "warn" });
    if (telemetry.odometry_status !== "online")
      a.push({ msg: "Odometría no disponible", tone: "warn" });
    if (telemetry.connection_status !== "connected")
      a.push({ msg: `Enlace ${telemetry.connection_status}`, tone: "warn" });
    return a;
  }, [
    telemetry.camera_status,
    telemetry.connection_status,
    telemetry.odometry_status,
    telemetry.velocity_status,
    telemetryState.online,
    topics.ros_available,
  ]);

  return (
    <div className="relative min-h-screen bg-background text-foreground overflow-hidden">
      <div className="ares-backdrop">
        <div className="ares-dots" style={{ right: 24, top: 24 }} />
        <div className="ares-dots" style={{ left: 24, bottom: 24, opacity: 0.4 }} />
        <div
          className="ares-diag-line"
          style={{ width: 420, top: 90, left: -40, transform: "rotate(35deg)" }}
        />
        <div
          className="ares-diag-line"
          style={{ width: 320, bottom: 120, right: -20, transform: "rotate(-35deg)" }}
        />
      </div>

      <div className="relative z-10 max-w-[1400px] mx-auto px-6 py-6 lg:px-10 lg:py-8">
        <header className="panel p-6 lg:p-8 mb-6 relative">
          <div className="absolute right-0 top-0 h-full w-40 bg-gradient-to-l from-mustard/15 to-transparent pointer-events-none" />
          <div className="flex flex-col lg:flex-row lg:items-center lg:justify-between gap-6">
            <div className="flex items-center gap-5">
              <div className="relative">
                <div className="h-16 w-16 rounded-md bg-mustard rotate-45 flex items-center justify-center">
                  <Cpu className="h-7 w-7 text-cocoa-deep -rotate-45" />
                </div>
                <div className="absolute -bottom-2 -right-2 h-6 w-6 bg-graphite rotate-45 border-2 border-background" />
              </div>
              <div>
                <div className="flex flex-wrap items-center gap-3 text-[0.7rem] uppercase tracking-[0.35em] text-mustard mb-1">
                  <span>Aisle</span>
                  <span>Rover</span>
                  <span>Environmental</span>
                  <span>Surveillance</span>
                </div>
                <h1 className="font-display text-5xl lg:text-6xl leading-none text-ivory">
                  A.R.E.S.
                </h1>
                <div className="mt-2 text-gold-soft tracking-wider">Equipo Wall-e</div>
              </div>
            </div>

            <div className="flex flex-wrap items-center gap-3">
              <div className="panel px-4 py-3 flex items-center gap-3">
                {telemetry.connection_status === "connected" ? (
                  <Wifi className="h-4 w-4 text-success" />
                ) : (
                  <WifiOff className="h-4 w-4 text-destructive" />
                )}
                <div>
                  <div className="text-[0.65rem] uppercase tracking-widest text-muted-foreground">
                    Conexión
                  </div>
                  <div className="text-sm font-semibold text-ivory capitalize">
                    {telemetry.connection_status}
                  </div>
                </div>
              </div>
              <div className="panel px-4 py-3">
                <div className="text-[0.65rem] uppercase tracking-widest text-muted-foreground">
                  ROS2 bridge
                </div>
                <div className="text-sm font-semibold text-ivory">
                  {topics.ros_available ? "activo" : "offline"}
                </div>
              </div>
              <div className="panel px-4 py-3">
                <div className="text-[0.65rem] uppercase tracking-widest text-muted-foreground">
                  Última actualización
                </div>
                <div className="text-sm font-semibold text-ivory tabular-nums">
                  {telemetry.last_update}
                </div>
              </div>
              <span className="chip chip-gold">SYS-01</span>
              <span className="chip chip-steel">v1.0</span>
            </div>
          </div>
        </header>

        {alerts.length > 0 && (
          <div className="panel p-3 mb-6 flex items-center gap-3 border-l-4 border-l-warning">
            <AlertTriangle className="h-5 w-5 text-warning shrink-0" />
            <div className="flex flex-wrap gap-x-6 gap-y-1 text-sm">
              {alerts.map((a, i) => (
                <span key={i} className="text-ivory/90">
                  <span className={`status-dot ${a.tone === "off" ? "off" : "warn"} mr-2`} />
                  {a.msg}
                </span>
              ))}
            </div>
          </div>
        )}

        <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
          <section className="xl:col-span-2 panel p-5">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-3">
                <Cctv className="h-5 w-5 text-mustard" />
                <h2 className="font-display text-2xl tracking-wide">Nodo de Cámara</h2>
                <span className="chip chip-gold">{cameraSocket.connected ? "WS" : "REST"}</span>
              </div>
              <StatusBadge status={toBadgeStatus(cameraStatus)} />
            </div>

            <div className="relative aspect-video rounded-md overflow-hidden border border-border bg-cocoa-deep">
              {cameraFrame?.data ? (
                <img src={cameraFrame.data} alt="RGB ROS2" className="w-full h-full object-cover" />
              ) : telemetry.camera_stream_url ? (
                <video
                  src={telemetry.camera_stream_url}
                  className="w-full h-full object-cover"
                  autoPlay
                  muted
                  loop
                  controls
                  playsInline
                />
              ) : (
                <div className="absolute inset-0 grid place-items-center">
                  <div className="text-center">
                    <Radio className="h-12 w-12 text-mustard mx-auto mb-3 pulse" />
                    <div className="font-display text-2xl text-ivory">SEÑAL EN ESPERA</div>
                    <div className="text-xs uppercase tracking-[0.3em] text-muted-foreground mt-1">
                      /ws/camera
                    </div>
                  </div>
                </div>
              )}
              <div className="absolute top-3 left-3 h-6 w-6 border-t-2 border-l-2 border-mustard" />
              <div className="absolute top-3 right-3 h-6 w-6 border-t-2 border-r-2 border-mustard" />
              <div className="absolute bottom-3 left-3 h-6 w-6 border-b-2 border-l-2 border-mustard" />
              <div className="absolute bottom-3 right-3 h-6 w-6 border-b-2 border-r-2 border-mustard" />
              <div className="absolute top-3 left-1/2 -translate-x-1/2 text-[0.65rem] uppercase tracking-[0.3em] text-mustard/80">
                A.R.E.S · RGB
              </div>
              {cameraSocket.packet.stale && (
                <div className="absolute bottom-3 left-1/2 -translate-x-1/2 bg-destructive/90 px-3 py-1 text-[0.65rem] uppercase tracking-[0.2em] text-ivory">
                  Frame stale · {formatAge(cameraSocket.packet.last_seen_age)}
                </div>
              )}
              <div className="scanline absolute inset-0" />
            </div>

            <div className="mt-4 grid grid-cols-1 sm:grid-cols-3 gap-3 text-xs">
              <InfoStat icon={ImageIcon} label="Resolución" value={cameraResolution} />
              <InfoStat icon={Database} label="Encoding" value={cameraFrame?.encoding ?? "--"} />
              <InfoStat icon={Network} label="Topic" value={cameraSocket.packet.topic || "--"} />
            </div>
          </section>

          <aside className="space-y-6">
            <div className="panel p-5 relative overflow-hidden">
              <div className="absolute -top-8 -right-8 w-32 h-32 bg-destructive/15 rounded-full blur-2xl" />
              <div className="flex items-center gap-2 mb-4">
                <Power className="h-5 w-5 text-destructive" />
                <h3 className="font-display text-xl tracking-wide">E-STOP</h3>
              </div>
              <button
                onClick={handleEstopClick}
                disabled={estopBusy}
                className={`group relative w-full aspect-square max-h-44 mx-auto rounded-full border-4 transition-all
                  ${
                    estopArmed
                      ? "bg-destructive border-destructive/70 shadow-[0_0_40px_rgba(220,50,50,0.45)]"
                      : "bg-cocoa border-destructive/60 hover:border-destructive"
                  } ${estopBusy ? "opacity-70 cursor-wait" : ""}`}
              >
                <div className="absolute inset-3 rounded-full border-2 border-dashed border-ivory/30 grid place-items-center">
                  <span className="font-display text-2xl tracking-widest text-ivory">
                    {estopBusy ? "WAIT" : estopArmed ? "STOPPED" : "PUSH"}
                  </span>
                </div>
              </button>
              <div className="text-center text-xs text-muted-foreground mt-3 uppercase tracking-widest">
                {estopArmed ? "Sistema detenido" : "Presiona para detener"}
              </div>
              {estopError && (
                <div className="mt-3 text-center text-xs text-destructive">{estopError}</div>
              )}
            </div>

            <div className="panel p-5">
              <div className="flex items-center gap-2 mb-3">
                <CircleDot className="h-5 w-5 text-mustard" />
                <h3 className="font-display text-xl tracking-wide">Estado de Nodos</h3>
              </div>
              <NodeRow
                name="Cámara"
                status={toBadgeStatus(cameraStatus) as NodeStatus}
                icon={Cctv}
                detail={cameraSocket.packet.topic || rtabmap.rgb_topic}
              />
              <NodeRow name="Velocidad" status={telemetry.velocity_status} icon={Gauge} />
              <NodeRow name="Odometría" status={telemetry.odometry_status} icon={MapPin} />
              <NodeRow
                name="RTAB-Map"
                status={toBadgeStatus(rtabmap.tracking_status) as NodeStatus}
                icon={Radar}
                detail={rtabmap.frame_id}
              />
            </div>
          </aside>

          <section className="xl:col-span-3">
            <div className="flex items-center gap-3 mb-4">
              <Activity className="h-5 w-5 text-mustard" />
              <h2 className="font-display text-2xl tracking-wide">Telemetría</h2>
              <div className="h-px flex-1 bg-border" />
            </div>
            <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-5 gap-4">
              <MetricCard
                icon={Gauge}
                label="Velocidad lineal"
                value={telemetry.linear_velocity.toFixed(2)}
                unit="m/s"
                tone="gold"
              />
              <MetricCard
                icon={Activity}
                label="Velocidad angular"
                value={telemetry.angular_velocity.toFixed(2)}
                unit="rad/s"
                tone="ivory"
              />
              <MetricCard
                icon={MapPin}
                label="Odometría X"
                value={telemetry.odom_x.toFixed(2)}
                unit="m"
                tone="steel"
              />
              <MetricCard
                icon={MapPin}
                label="Odometría Y"
                value={telemetry.odom_y.toFixed(2)}
                unit="m"
                tone="steel"
              />
              <MetricCard
                icon={Compass}
                label="Yaw"
                value={telemetry.odom_yaw.toFixed(2)}
                unit="rad"
                tone="gold"
              />
            </div>
          </section>

          <section className="xl:col-span-3 panel p-5">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-3">
                <Activity className="h-5 w-5 text-mustard" />
                <h3 className="font-display text-xl tracking-wide">Velocidad lineal - histórico</h3>
              </div>
              <span className="chip chip-ivory">m/s</span>
            </div>
            <div className="h-56">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={velData} margin={{ top: 10, right: 16, bottom: 0, left: -10 }}>
                  <CartesianGrid stroke="oklch(0.32 0.02 55)" strokeDasharray="3 6" />
                  <XAxis
                    dataKey="t"
                    stroke="oklch(0.6 0.01 70)"
                    tick={{ fontSize: 11 }}
                    domain={[0, 19]}
                    type="number"
                  />
                  <YAxis stroke="oklch(0.6 0.01 70)" tick={{ fontSize: 11 }} />
                  <Tooltip
                    contentStyle={{
                      background: "oklch(0.22 0.025 55)",
                      border: "1px solid oklch(0.32 0.02 55)",
                      borderRadius: 8,
                      color: "oklch(0.96 0.012 85)",
                    }}
                  />
                  <Line
                    type="linear"
                    dataKey="v"
                    stroke="var(--mustard)"
                    strokeWidth={2.5}
                    dot={false}
                    isAnimationActive={false}
                    activeDot={{ r: 4, fill: "var(--gold-soft)" }}
                  />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </section>

          <section className="xl:col-span-3">
            <div className="flex items-center gap-3 mb-4">
              <Network className="h-5 w-5 text-mustard" />
              <h2 className="font-display text-2xl tracking-wide">ROS2 Bridge</h2>
              <div className="h-px flex-1 bg-border" />
              <StatusBadge status={topics.ros_available ? "online" : "offline"} />
            </div>
            <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
              <div className="panel p-5 lg:col-span-2">
                <div className="flex items-center justify-between mb-3">
                  <div className="flex items-center gap-2">
                    <Radio className="h-5 w-5 text-mustard" />
                    <h3 className="font-display text-xl tracking-wide">Topics ROS2</h3>
                  </div>
                  <span className="chip chip-steel">{topics.topics.length}</span>
                </div>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-x-5">
                  {highlightedTopics.map((topic) => (
                    <TopicRow key={topic.name} topic={topic} />
                  ))}
                  {highlightedTopics.length === 0 && (
                    <div className="text-sm text-muted-foreground">Sin topics disponibles</div>
                  )}
                </div>
                {topics.error && (
                  <div className="mt-3 text-xs text-destructive truncate">{topics.error}</div>
                )}
              </div>

              <div className="panel p-5">
                <div className="flex items-center justify-between mb-3">
                  <div className="flex items-center gap-2">
                    <Radar className="h-5 w-5 text-mustard" />
                    <h3 className="font-display text-xl tracking-wide">RTAB-Map</h3>
                  </div>
                  <StatusBadge status={toBadgeStatus(rtabmap.tracking_status)} />
                </div>
                <div className="grid grid-cols-2 gap-3">
                  <InfoStat icon={Cctv} label="RGB" value={rtabmap.sources.rgb ?? "offline"} />
                  <InfoStat
                    icon={MapPin}
                    label="Odom"
                    value={rtabmap.visual_odometry ? "online" : "offline"}
                  />
                  <InfoStat icon={Activity} label="IMU" value={rtabmap.sources.imu ?? "offline"} />
                </div>
                <div className="mt-4 space-y-2 text-xs text-muted-foreground">
                  <div className="truncate">RGB: {rtabmap.rgb_topic}</div>
                  <div className="truncate">IMU: {rtabmap.imu_topic}</div>
                  <div className="truncate">
                    Error: {rtabmapState.error ?? topicsState.error ?? "--"}
                  </div>
                </div>
              </div>
            </div>
          </section>
        </div>

        <footer className="mt-8 flex flex-col sm:flex-row gap-2 sm:items-center sm:justify-between text-xs text-muted-foreground">
          <div className="tracking-widest uppercase">
            A.R.E.S · Aisle Rover Environmental Surveillance
          </div>
          <div className="tracking-widest uppercase">Equipo Wall-e · Mission Control</div>
        </footer>
      </div>
    </div>
  );
}
