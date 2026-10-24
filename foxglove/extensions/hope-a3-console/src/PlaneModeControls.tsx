import { ReactElement, useState } from "react";

import { PlaneMode, PlaneModeState, PLANE_MODES, PLANE_MODE_LABELS } from "./planeMode";

export function PlaneModeControls({ state, available, onApply }: {
  state?: PlaneModeState; available: boolean; onApply: (mode: PlaneMode) => Promise<void>;
}): ReactElement {
  const [pending, setPending] = useState(false);
  const [error, setError] = useState("");
  const change = async (mode: PlaneMode) => {
    setPending(true); setError("");
    try { await onApply(mode); }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setPending(false); }
  };
  const metres = (value?: number) => value == undefined ? "—" : `${value.toFixed(3)} m`;
  return <div className="plane-mode-controls">
    <label>击球平面 · 下一球
      <select aria-label="击球平面模式" disabled={!available || pending || state == undefined}
        value={state?.requested ?? ""} onChange={(event) => void change(event.target.value as PlaneMode)}>
        {state == undefined && <option value="">等待 Planner 状态</option>}
        {PLANE_MODES.map((mode) => <option key={mode} value={mode}>{PLANE_MODE_LABELS[mode]}</option>)}
      </select>
    </label>
    <div>START 后可切换；本球已发目标保持不变。重启 Planner 后恢复启动配置。</div>
    {pending && <div role="status">等待 Planner 确认…</div>}
    {state != undefined && <>
      <div>最近已发球 #{state.flight > 0 ? state.flight : "—"}：{state.lastCommand == undefined ? "尚未发球" : PLANE_MODE_LABELS[state.lastCommand]}</div>
      <div>参考 X {metres(state.referenceX)} · 平面 X {metres(state.planeX)} · policy 偏移 {metres(state.offset)}</div>
      {state.requested !== state.lastCommand && <div>所选模式将在后续新球采用。</div>}
      {state.requested !== "fixed" && !state.supported && <div>等待支持动态平面的 Runner 会话；尚不能确认动态模式可用。</div>}
      {state.requested === "stable_per_ball" && <div>连续 200 ms：等待中、水平速度 ≤3 cm/s、角速度 ≤0.2 rad/s、倾斜 ≤5°。不含足底承重判定。{state.stable ? "当前满足姿态判据。" : "未满足时沿用上一平面。"}</div>}
    </>}
    {error && <div role="alert">{error}</div>}
  </div>;
}
