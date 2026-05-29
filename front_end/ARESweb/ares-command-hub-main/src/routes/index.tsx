import { createFileRoute } from "@tanstack/react-router";
import AresDashboard from "@/components/ares/AresDashboard";

export const Route = createFileRoute("/")({
  head: () => ({
    meta: [
      { title: "A.R.E.S. — Equipo Wall-e | Dashboard de Supervisión" },
      {
        name: "description",
        content:
          "Dashboard de supervisión del rover autónomo A.R.E.S. (Aisle Rover Environmental Surveillance) — Equipo Wall-e.",
      },
    ],
  }),
  component: Index,
});

function Index() {
  return <AresDashboard />;
}
