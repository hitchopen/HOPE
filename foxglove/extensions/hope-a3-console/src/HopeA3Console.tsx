import "@fontsource/ibm-plex-mono/400.css";
import "@fontsource/ibm-plex-mono/500.css";
import "@fontsource/ibm-plex-mono/600.css";
import "@fontsource/ibm-plex-sans/400.css";
import "@fontsource/ibm-plex-sans/500.css";
import "@fontsource/ibm-plex-sans/600.css";
import "@fontsource/space-grotesk/500.css";
import "@fontsource/space-grotesk/600.css";
import "@fontsource/space-grotesk/700.css";

import { Immutable, MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

import "./styles.css";

const TOPICS = {
  ntpOffset: "/hope/ntp/offset_ms",
  ntpDispersion: "/hope/ntp/root_dispersion_ms",
  ntpGate: "/hope/ntp/gate_pass",
  messageLatency: "/hope/clock/message_latency_ms",
  messageFresh: "/hope/clock/message_fresh",
  cpu: "/hope/system/cpu_load_percent",
  agibotPm: "/hope/vendor/agibot_pm_active",
  tfReady: "/hope/vendor/tf_ready",
  estopReady: "/hope/safety/estop_ready",
} as const;

const ESTOP_SERVICE = "/hope/safety/trigger_estop";
const FAST_STALE_MS = 500;
const SLOW_STALE_MS = 2500;
const CPU_WINDOW_MS = 120_000;
const CPU_MAX_SAMPLES = 600;

type TopicKey = keyof typeof TOPICS;
type Scalar = boolean | number;

interface Datum {
  value?: Scalar;
  receivedAt?: number;
}

type DashboardData = Record<TopicKey, Datum>;

interface CpuSample {
  time: number;
  value: number;
}

const EMPTY_DATA: DashboardData = {
  ntpOffset: {},
  ntpDispersion: {},
  ntpGate: {},
  messageLatency: {},
  messageFresh: {},
  cpu: {},
  agibotPm: {},
  tfReady: {},
  estopReady: {},
};

const TOPIC_TO_KEY = new Map<string, TopicKey>(
  Object.entries(TOPICS).map(([key, topic]) => [topic, key as TopicKey]),
);

function scalarFromMessage(message: unknown): Scalar | undefined {
  if (typeof message !== "object" || message == undefined || !("data" in message)) {
    return undefined;
  }
  const value = (message as { data?: unknown }).data;
  if (typeof value === "boolean") {
    return value;
  }
  if (typeof value === "number" && Number.isFinite(value)) {
    return value;
  }
  return undefined;
}

function isFresh(datum: Datum, now: number, staleMs: number): boolean {
  return datum.receivedAt != undefined && now - datum.receivedAt <= staleMs;
}

function numberValue(datum: Datum): number | undefined {
  return typeof datum.value === "number" ? datum.value : undefined;
}

function boolValue(datum: Datum): boolean | undefined {
  return typeof datum.value === "boolean" ? datum.value : undefined;
}

function formatNumber(value: number | undefined, digits: number): string {
  return value == undefined ? "—" : value.toFixed(digits);
}

type HealthTone = "good" | "attention" | "unknown";

function healthTone({ value, fresh }: { value: boolean | undefined; fresh: boolean }): HealthTone {
  if (!fresh || value == undefined) {
    return "unknown";
  }
  return value ? "good" : "attention";
}

function GateChip({
  label,
  value,
  fresh,
}: {
  label: string;
  value: boolean | undefined;
  fresh: boolean;
}): ReactElement {
  const tone = healthTone({ value, fresh });
  const suffix = tone === "unknown" ? " · NO DATA" : value === true ? "" : " · ATTENTION";
  return (
    <div className={`gate-chip gate-chip--${tone}`}>
      <span className="status-dot" />
      {label}
      {suffix}
    </div>
  );
}

function NumericCard({
  eyebrow,
  value,
  digits,
  unit,
  topic,
  footer,
  fresh,
}: {
  eyebrow: string;
  value: number | undefined;
  digits: number;
  unit: string;
  topic: string;
  footer?: string;
  fresh: boolean;
}): ReactElement {
  return (
    <section className="metric-card">
      <div className="eyebrow">{eyebrow}</div>
      <div className={`metric-value-row${fresh ? "" : " metric-value-row--stale"}`}>
        <span className="metric-value">{formatNumber(value, digits)}</span>
        <span className="metric-unit">{unit}</span>
      </div>
      <div className="metric-footer">
        {topic}
        {footer}
        {!fresh && " · stale"}
      </div>
    </section>
  );
}

function ProcessTile({
  name,
  value,
  fresh,
  topic,
}: {
  name: string;
  value?: boolean;
  fresh: boolean;
  topic?: string;
}): ReactElement {
  const state = !fresh || value == undefined ? "unknown" : value ? "running" : "stopped";
  const label = state === "running" ? "RUNNING" : state === "stopped" ? "STOPPED" : "NO DATA";
  return (
    <section className={`process-tile process-tile--${state}`}>
      <div className="process-status">
        <span className="process-dot" />
        <span>{label}</span>
      </div>
      <div className="process-name">{name}</div>
      <div className="process-topic">{topic ?? ""}</div>
    </section>
  );
}

function CpuPlot({ samples, now }: { samples: CpuSample[]; now: number }): ReactElement {
  const points = useMemo(() => {
    const start = now - CPU_WINDOW_MS;
    return samples
      .filter((sample) => sample.time >= start)
      .map((sample) => {
        const x = ((sample.time - start) / CPU_WINDOW_MS) * 600;
        const y = 150 - Math.min(100, Math.max(0, sample.value)) * 1.5;
        return `${x.toFixed(1)},${y.toFixed(1)}`;
      });
  }, [samples, now]);
  const line = points.join(" ");
  const fill = points.length > 0 ? `0,150 ${line} 600,150` : "";
  return (
    <svg className="cpu-plot" viewBox="0 0 600 150" preserveAspectRatio="none" role="img">
      <title>A3 CPU load over the last 120 seconds</title>
      <line x1="0" y1="0" x2="600" y2="0" className="cpu-grid" />
      <line x1="0" y1="75" x2="600" y2="75" className="cpu-grid" />
      <line x1="0" y1="149" x2="600" y2="149" className="cpu-grid" />
      {fill && <polygon points={fill} className="cpu-fill" />}
      {line && <polyline points={line} className="cpu-line" />}
    </svg>
  );
}

function HopeA3Console({ context }: { context: PanelExtensionContext }): ReactElement {
  const [data, setData] = useState<DashboardData>(EMPTY_DATA);
  const [cpuSamples, setCpuSamples] = useState<CpuSample[]>([]);
  const [now, setNow] = useState(() => Date.now());
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [estopInFlight, setEstopInFlight] = useState(false);
  const [estopMessage, setEstopMessage] = useState("");

  useLayoutEffect(() => {
    context.setDefaultPanelTitle("HOPE A3 Console");
    context.onRender = (renderState, done) => {
      const receivedAt = Date.now();
      const frame: Immutable<MessageEvent[]> = renderState.currentFrame ?? [];
      if (frame.length > 0) {
        const updates = new Map<TopicKey, Scalar>();
        for (const event of frame) {
          const key = TOPIC_TO_KEY.get(event.topic);
          if (key == undefined) {
            continue;
          }
          const value = scalarFromMessage(event.message);
          if (value != undefined) {
            updates.set(key, value);
          }
        }
        if (updates.size > 0) {
          setData((previous) => {
            const next = { ...previous };
            for (const [key, value] of updates) {
              next[key] = { value, receivedAt };
            }
            return next;
          });
          const cpu = updates.get("cpu");
          if (typeof cpu === "number") {
            setCpuSamples((previous) =>
              [...previous, { time: receivedAt, value: cpu }]
                .filter((sample) => sample.time >= receivedAt - CPU_WINDOW_MS)
                .slice(-CPU_MAX_SAMPLES),
            );
          }
        }
      }
      setRenderDone(() => done);
    };
    context.watch("currentFrame");
    context.subscribe(Object.values(TOPICS).map((topic) => ({ topic })));
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      setNow(Date.now());
    }, 250);
    return () => {
      window.clearInterval(timer);
    };
  }, []);

  const anySourceFresh = Object.values(data).some((datum) => isFresh(datum, now, SLOW_STALE_MS));
  const ntpFresh = isFresh(data.ntpOffset, now, SLOW_STALE_MS);
  const dispersionFresh = isFresh(data.ntpDispersion, now, SLOW_STALE_MS);
  const latencyFresh = isFresh(data.messageLatency, now, FAST_STALE_MS);
  const cpuFresh = isFresh(data.cpu, now, SLOW_STALE_MS);
  const pmFresh = isFresh(data.agibotPm, now, SLOW_STALE_MS);
  const estopReadyFresh = isFresh(data.estopReady, now, SLOW_STALE_MS);
  const estopReady = estopReadyFresh && boolValue(data.estopReady) === true;

  const assertEstop = useCallback(async () => {
    if (!estopReady || estopInFlight) {
      return;
    }
    if (context.callService == undefined) {
      setEstopMessage("E-STOP FAILED · data source has no service-call support");
      return;
    }
    setEstopInFlight(true);
    setEstopMessage("ASSERTING SOFTWARE E-STOP…");
    try {
      const timeout = new Promise<never>((_resolve, reject) => {
        window.setTimeout(() => {
          reject(new Error("service call timed out after 3 s"));
        }, 3000);
      });
      const response = (await Promise.race([context.callService(ESTOP_SERVICE, {}), timeout])) as {
        success?: unknown;
        message?: unknown;
      };
      const message = typeof response.message === "string" ? response.message : "";
      if (response.success !== true) {
        throw new Error(message || "vendor success was not confirmed");
      }
      setEstopMessage(`E-STOP ASSERTED${message ? ` · ${message}` : ""}`);
    } catch (error) {
      const detail = error instanceof Error ? error.message : String(error);
      setEstopMessage(`E-STOP FAILED · ${detail}`);
    } finally {
      setEstopInFlight(false);
    }
  }, [context, estopInFlight, estopReady]);

  const dispersion = numberValue(data.ntpDispersion);
  const ntpFooter =
    dispersionFresh && dispersion != undefined ? ` · disp ${dispersion.toFixed(2)}` : "";
  const cpu = numberValue(data.cpu);

  return (
    <main className="hope-console">
      <div className="source-estop-row">
        <div className="source-block">
          <div className="eyebrow">A3 SOURCE</div>
          <div className="source-value">
            <span>FOXGLOVE WEBSOCKET</span>
            <span className={`source-dot${anySourceFresh ? " source-dot--live" : ""}`} />
          </div>
          <div className="source-helper">
            {anySourceFresh ? "live A3 monitor data" : "waiting for data"} · address set in
            connection dialog
          </div>
        </div>
        <button
          type="button"
          className="estop-button"
          disabled={!estopReady || estopInFlight}
          onClick={() => void assertEstop()}
        >
          <span>{estopInFlight ? "ASSERTING…" : "E-STOP"}</span>
          <small>ASSERT ONLY · NO RESET</small>
        </button>
      </div>

      <div className="estop-result" aria-live="polite">
        {estopMessage}
      </div>

      <div className="gate-row">
        <GateChip
          label="NTP GATE"
          value={boolValue(data.ntpGate)}
          fresh={isFresh(data.ntpGate, now, SLOW_STALE_MS)}
        />
        <GateChip
          label="TIMESTAMP"
          value={boolValue(data.messageFresh)}
          fresh={isFresh(data.messageFresh, now, FAST_STALE_MS)}
        />
        <GateChip
          label="TF READY"
          value={boolValue(data.tfReady)}
          fresh={isFresh(data.tfReady, now, SLOW_STALE_MS)}
        />
        <GateChip
          label="E-STOP BACKEND"
          value={boolValue(data.estopReady)}
          fresh={estopReadyFresh}
        />
      </div>

      <div className="metric-grid">
        <NumericCard
          eyebrow="NTP WORLD-CLOCK OFFSET"
          value={numberValue(data.ntpOffset)}
          digits={2}
          unit="ms"
          topic={TOPICS.ntpOffset}
          footer={ntpFooter}
          fresh={ntpFresh}
        />
        <NumericCard
          eyebrow="ROS 2 MESSAGE LATENCY"
          value={numberValue(data.messageLatency)}
          digits={1}
          unit="ms"
          topic={TOPICS.messageLatency}
          fresh={latencyFresh}
        />
      </div>

      <div className="process-grid">
        <ProcessTile
          name="agibot_pm"
          value={boolValue(data.agibotPm)}
          fresh={pmFresh}
          topic={TOPICS.agibotPm}
        />
        <ProcessTile name="HDU" fresh={false} />
        <ProcessTile name="MDU" fresh={false} />
      </div>

      <section className="cpu-card">
        <div className="cpu-header">
          <span className="eyebrow">A3 CPU LOAD</span>
          <span className={`cpu-value${cpuFresh ? "" : " cpu-value--stale"}`}>
            {formatNumber(cpu, 0)}
            <small>%</small>
          </span>
          <span className="cpu-topic">
            {TOPICS.cpu} · 0–100 %, 120 s{!cpuFresh && " · stale"}
          </span>
        </div>
        <CpuPlot samples={cpuSamples} now={now} />
      </section>

      <section className="sequence-card" aria-label="Sequence controls not implemented">
        <div className="eyebrow">SEQUENCE</div>
        <div className="sequence-empty" />
      </section>
    </main>
  );
}

export function initHopeA3Console(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HopeA3Console context={context} />);
  return () => {
    root.unmount();
  };
}
