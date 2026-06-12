import { useEffect, useMemo, useState } from "react";
import {
  Activity,
  Cctv,
  Gauge,
  Compass,
  MapPin,
  Wifi,
  WifiOff,
  AlertTriangle,
  Power,
  Radio,
  Cpu,
  CircleDot,
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

function StatusBadge({ status }: { status: NodeStatus | "connected" | "offline" | "degraded" }) {
  const map = {
    online: { dot: "on", label: "Online" },
    connected: { dot: "on", label: "Conectado" },
    degraded: { dot: "warn", label: "Degradado" },
    offline: { dot: "off", label: "Offline" },
  } as const;
  const s = map[status as keyof typeof map];
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
  icon: React.ComponentType<{ className?: string }>;
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

function NodeRow({ name, status, icon: Icon }: { name: string; status: NodeStatus; icon: React.ComponentType<{ className?: string }> }) {
  return (
    <div className="flex items-center justify-between py-3 border-b border-border last:border-0">
      <div className="flex items-center gap-3">
        <div className="h-9 w-9 rounded-md bg-cocoa flex items-center justify-center border border-border">
          <Icon className="h-4 w-4 text-mustard" />
        </div>
        <div>
          <div className="font-medium text-ivory">{name}</div>
          <div className="text-xs text-muted-foreground">/ros2/node</div>
        </div>
      </div>
      <StatusBadge status={status} />
    </div>
  );
}

function useVelocityHistory(current: number) {
  const [data, setData] = useState(() =>
    Array.from({ length: 20 }, (_, i) => ({ t: i, v: current }))
  );
  useEffect(() => {
    const id = setInterval(() => {
      setData((prev) => {
        const next = [...prev.slice(1), { t: prev[prev.length - 1].t + 1, v: +(current + (Math.random() - 0.5) * 0.15).toFixed(3) }];
        return next;
      });
    }, 1200);
    return () => clearInterval(id);
  }, [current]);
  return data;
}

export default function AresDashboard() {
  const [telemetry, setTelemetry] = useState<Telemetry>(DEFAULT_TELEMETRY);
  const [estopArmed, setEstopArmed] = useState(false);

  useEffect(() => {
    const fetchTelemetry = async () => {
      try {
        const res = await fetch("http://127.0.0.1:8000/api/telemetry");
        const data = await res.json();
        setTelemetry(data);
      } catch (error) {
        console.error("Error cargando telemetría:", error);
      }
    };

    fetchTelemetry();
    const id = setInterval(fetchTelemetry, 1000);

    return () => clearInterval(id);
  }, []);

  const velData = useVelocityHistory(telemetry.linear_velocity);

  const alerts = useMemo(() => {
    const a: { msg: string; tone: "warn" | "off" }[] = [];
    if (telemetry.camera_status !== "online") a.push({ msg: "Nodo de cámara sin transmisión", tone: "off" });
    if (telemetry.velocity_status !== "online") a.push({ msg: "Nodo de velocidad inestable", tone: "warn" });
    if (telemetry.odometry_status !== "online") a.push({ msg: "Odometría no disponible", tone: "warn" });
    if (telemetry.connection_status !== "connected") a.push({ msg: `Enlace ${telemetry.connection_status}`, tone: "warn" });
    return a;
  }, []);

  return (
    <div className="relative min-h-screen bg-background text-foreground overflow-hidden">
      {/* Decorative geometric backdrop */}
      <div className="ares-backdrop">
        <div className="ares-dots" style={{ right: 24, top: 24 }} />
        <div className="ares-dots" style={{ left: 24, bottom: 24, opacity: 0.4 }} />
        <div className="ares-diag-line" style={{ width: 420, top: 90, left: -40, transform: "rotate(35deg)" }} />
        <div className="ares-diag-line" style={{ width: 320, bottom: 120, right: -20, transform: "rotate(-35deg)" }} />
      </div>

      <div className="relative z-10 max-w-[1400px] mx-auto px-6 py-6 lg:px-10 lg:py-8">
        {/* HEADER */}
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
                <div className="flex items-center gap-3 text-[0.7rem] uppercase tracking-[0.35em] text-mustard mb-1">
                  <span>Aisle</span><span>Rover</span><span>Environmental</span><span>Surveillance</span>
                </div>
                <h1 className="font-display text-5xl lg:text-6xl leading-none text-ivory">A.R.E.S.</h1>
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
                  <div className="text-[0.65rem] uppercase tracking-widest text-muted-foreground">Conexión</div>
                  <div className="text-sm font-semibold text-ivory capitalize">{telemetry.connection_status}</div>
                </div>
              </div>
              <div className="panel px-4 py-3">
                <div className="text-[0.65rem] uppercase tracking-widest text-muted-foreground">Última actualización</div>
                <div className="text-sm font-semibold text-ivory tabular-nums">{telemetry.last_update}</div>
              </div>
              <span className="chip chip-gold">SYS-01</span>
              <span className="chip chip-steel">v1.0</span>
            </div>
          </div>
        </header>

        {/* ALERTS */}
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

        {/* MAIN GRID */}
        <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
          {/* CAMERA */}
          <section className="xl:col-span-2 panel p-5">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-3">
                <Cctv className="h-5 w-5 text-mustard" />
                <h2 className="font-display text-2xl tracking-wide">Nodo de Cámara</h2>
                <span className="chip chip-gold">LIVE</span>
              </div>
              <StatusBadge status={telemetry.camera_status} />
            </div>

            <div className="relative aspect-video rounded-md overflow-hidden border border-border bg-cocoa-deep">
              {telemetry.camera_stream_url ? (
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
                      camera_stream_url
                    </div>
                  </div>
                </div>
              )}
              {/* HUD corners */}
              <div className="absolute top-3 left-3 h-6 w-6 border-t-2 border-l-2 border-mustard" />
              <div className="absolute top-3 right-3 h-6 w-6 border-t-2 border-r-2 border-mustard" />
              <div className="absolute bottom-3 left-3 h-6 w-6 border-b-2 border-l-2 border-mustard" />
              <div className="absolute bottom-3 right-3 h-6 w-6 border-b-2 border-r-2 border-mustard" />
              <div className="absolute top-3 left-1/2 -translate-x-1/2 text-[0.65rem] uppercase tracking-[0.3em] text-mustard/80">
                A.R.E.S · CAM-00
              </div>
              <div className="scanline absolute inset-0" />
            </div>

            <div className="mt-4 grid grid-cols-3 gap-3 text-xs">
              <div className="panel p-3">
                <div className="text-muted-foreground uppercase tracking-widest text-[0.65rem]">Resolución</div>
                <div className="font-display text-lg">1280×720</div>
              </div>
              <div className="panel p-3">
                <div className="text-muted-foreground uppercase tracking-widest text-[0.65rem]">FPS</div>
                <div className="font-display text-lg">30</div>
              </div>
              <div className="panel p-3">
                <div className="text-muted-foreground uppercase tracking-widest text-[0.65rem]">Códec</div>
                <div className="font-display text-lg">MJPEG</div>
              </div>
            </div>
          </section>

          {/* E-STOP + NODES */}
          <aside className="space-y-6">
            <div className="panel p-5 relative overflow-hidden">
              <div className="absolute -top-8 -right-8 w-32 h-32 bg-destructive/15 rounded-full blur-2xl" />
              <div className="flex items-center gap-2 mb-4">
                <Power className="h-5 w-5 text-destructive" />
                <h3 className="font-display text-xl tracking-wide">E-STOP</h3>
              </div>
              <button
                onClick={() => setEstopArmed((v) => !v)}
                className={`group relative w-full aspect-square max-h-44 mx-auto rounded-full border-4 transition-all
                  ${estopArmed
                    ? "bg-destructive border-destructive/70 shadow-[0_0_40px_rgba(220,50,50,0.45)]"
                    : "bg-cocoa border-destructive/60 hover:border-destructive"}`}
              >
                <div className="absolute inset-3 rounded-full border-2 border-dashed border-ivory/30 grid place-items-center">
                  <span className="font-display text-2xl tracking-widest text-ivory">
                    {estopArmed ? "STOPPED" : "PUSH"}
                  </span>
                </div>
              </button>
              <div className="text-center text-xs text-muted-foreground mt-3 uppercase tracking-widest">
                {estopArmed ? "Sistema detenido" : "Presiona para detener"}
              </div>
            </div>

            <div className="panel p-5">
              <div className="flex items-center gap-2 mb-3">
                <CircleDot className="h-5 w-5 text-mustard" />
                <h3 className="font-display text-xl tracking-wide">Estado de Nodos</h3>
              </div>
              <NodeRow name="Cámara" status={telemetry.camera_status} icon={Cctv} />
              <NodeRow name="Velocidad" status={telemetry.velocity_status} icon={Gauge} />
              <NodeRow name="Odometría" status={telemetry.odometry_status} icon={MapPin} />
            </div>
          </aside>

          {/* TELEMETRY */}
          <section className="xl:col-span-3">
            <div className="flex items-center gap-3 mb-4">
              <Activity className="h-5 w-5 text-mustard" />
              <h2 className="font-display text-2xl tracking-wide">Telemetría</h2>
              <div className="h-px flex-1 bg-border" />
            </div>
            <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-5 gap-4">
              <MetricCard icon={Gauge}   label="Velocidad lineal"  value={telemetry.linear_velocity.toFixed(2)}  unit="m/s"  tone="gold" />
              <MetricCard icon={Activity} label="Velocidad angular" value={telemetry.angular_velocity.toFixed(2)} unit="rad/s" tone="ivory" />
              <MetricCard icon={MapPin}   label="Odometría X"       value={telemetry.odom_x.toFixed(2)}            unit="m"     tone="steel" />
              <MetricCard icon={MapPin}   label="Odometría Y"       value={telemetry.odom_y.toFixed(2)}            unit="m"     tone="steel" />
              <MetricCard icon={Compass}  label="Yaw"               value={telemetry.odom_yaw.toFixed(2)}          unit="rad"   tone="gold" />
            </div>
          </section>

          {/* CHART */}
          <section className="xl:col-span-3 panel p-5">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-3">
                <Activity className="h-5 w-5 text-mustard" />
                <h3 className="font-display text-xl tracking-wide">Velocidad lineal — histórico</h3>
              </div>
              <span className="chip chip-ivory">m/s</span>
            </div>
            <div className="h-56">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={velData} margin={{ top: 10, right: 16, bottom: 0, left: -10 }}>
                  <CartesianGrid stroke="oklch(0.32 0.02 55)" strokeDasharray="3 6" />
                  <XAxis dataKey="t" stroke="oklch(0.6 0.01 70)" tick={{ fontSize: 11 }} />
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
                    type="monotone"
                    dataKey="v"
                    stroke="var(--mustard)"
                    strokeWidth={2.5}
                    dot={false}
                    activeDot={{ r: 4, fill: "var(--gold-soft)" }}
                  />
                </LineChart>
              </ResponsiveContainer>
            </div>
          </section>
        </div>

        <footer className="mt-8 flex items-center justify-between text-xs text-muted-foreground">
          <div className="tracking-widest uppercase">A.R.E.S · Aisle Rover Environmental Surveillance</div>
          <div className="tracking-widest uppercase">Equipo Wall-e · Mission Control</div>
        </footer>
      </div>
    </div>
  );
}
