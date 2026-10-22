export type BallModelStatus = {
  state: string;
  configured_model: string | null;
  age_s: number | null;
  endpoints: Record<string, { configured_model?: string | null; runtime_model?: string | null;
    runtime: string; mismatches?: string[]; error?: string }>;
};

export function ballModelDisplay(status: BallModelStatus | undefined, fieldFresh: boolean) {
  if (!fieldFresh || status == undefined || status.age_s == null || !Number.isFinite(status.age_s) || status.age_s > 45) {
    return { value: "", text: "等待两端最新参数核验", ready: false };
  }
  const labels: Record<string, string> = { LOADED: "两端已加载且一致", CONFIGURED: "两端配置一致（未全部运行）",
    MISMATCH: "参数不一致 · 禁止正常启动", UNAVAILABLE: "核验不可用", CHECKING: "正在核验" };
  const parts = [["hdu", "Planner"], ["laptop", "球包器"]].map(([key, label]) => {
    const e = status.endpoints[key!];
    if (e == undefined) { return `${label}：无回执`; }
    const model = e.configured_model?.toUpperCase() ?? "未知";
    const runtime = e.runtime === "VERIFIED" ? `已加载 ${e.runtime_model?.toUpperCase() ?? model}`
      : e.runtime === "NOT_RUNNING" ? "未运行" : e.runtime === "MISMATCH"
        ? `运行参数不匹配（${e.runtime_model?.toUpperCase() ?? "未知模型"}）` : "无法读取";
    return `${label}：配置 ${model}，${runtime}${e.error ? ` · ${e.error}` : ""}`;
  });
  const ready = ["LOADED", "CONFIGURED"].includes(status.state);
  return { value: status.configured_model ?? "", ready,
    text: `${labels[status.state] ?? "未知核验状态"} · ${parts.join("；")}` };
}
