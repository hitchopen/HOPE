import { ReactElement, useEffect, useState } from "react";

import { BallYFilterState, DEFAULT_BALL_Y_FILTER, parseBallYBounds } from "./ballYFilter";
import { BallYFilterDraft, ballYFilterDraftEdited, readBallYFilterDraft } from "./ballYFilterPreview";

const displayY = (value: number) => String(Number(value.toFixed(6)));

export function BallYFilterControls({ state, available, kernelMode, onApply, initialState, onPreview }: {
  state: BallYFilterState | undefined; available: boolean; kernelMode: boolean;
  onApply: (config: typeof DEFAULT_BALL_Y_FILTER) => Promise<void>;
  initialState?: unknown;
  onPreview: (draft: BallYFilterDraft, options: { kernelMode: boolean; edited: boolean }) => void;
}): ReactElement {
  const [initial] = useState(() => readBallYFilterDraft(initialState));
  const [enabled, setEnabled] = useState(initial.enabled);
  const [min, setMin] = useState(initial.min);
  const [max, setMax] = useState(initial.max);
  const [dirty, setDirty] = useState(() => ballYFilterDraftEdited(initialState));
  const [pending, setPending] = useState(false);
  const [error, setError] = useState("");
  const [previewError, setPreviewError] = useState("");
  useEffect(() => {
    try { onPreview({ enabled, min, max }, { kernelMode, edited: dirty }); setPreviewError(""); }
    catch (cause) { setPreviewError(`本地预览更新失败：${String(cause)}`); }
  }, [enabled, min, max, kernelMode, dirty, onPreview]);
  useEffect(() => {
    if (!dirty && state != undefined) {
      setEnabled(state.enabled); setMin(displayY(state.min)); setMax(displayY(state.max));
    }
  }, [dirty, state]);
  const bounds = parseBallYBounds(min, max);
  const edit = () => { setDirty(true); setError(""); };
  const apply = async () => {
    if (bounds == undefined || !available || pending) { return; }
    setPending(true); setError("");
    try { await onApply({ enabled, ...bounds }); setDirty(false); }
    catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)); }
    finally { setPending(false); }
  };
  return <div className="ball-y-filter" aria-label="Planner y filter">
    <label className="ball-y-filter-toggle">
      <input type="checkbox" checked={enabled} disabled={pending || kernelMode}
        onChange={(event) => { edit(); setEnabled(event.target.checked); }} />
      球路 y 过滤
      <span className={state?.enabled === true ? "filter-on" : "filter-off"}>
        {state == undefined ? "Planner 未连接" : state.enabled ? "Planner 已开启" : "Planner 已关闭"}
      </span>
    </label>
    <div className="ball-y-filter-inputs">
      <label>左边界 y (m)<input type="number" step="0.01" value={max} disabled={pending || kernelMode}
        aria-label="左边界 y (m)" onChange={(event) => { edit(); setMax(event.target.value); }} /></label>
      <label>右边界 y (m)<input type="number" step="0.01" value={min} disabled={pending || kernelMode}
        aria-label="右边界 y (m)" onChange={(event) => { edit(); setMin(event.target.value); }} /></label>
    </div>
    <div className="ball-y-filter-actions">
      <button type="button" disabled={pending || !available || bounds == undefined || !dirty}
        onClick={() => void apply()}>{pending ? "应用中…" : "应用到 Planner"}</button>
      <button type="button" disabled={pending || kernelMode} onClick={() => {
        edit(); setEnabled(DEFAULT_BALL_Y_FILTER.enabled);
        setMin(displayY(DEFAULT_BALL_Y_FILTER.min)); setMax(displayY(DEFAULT_BALL_Y_FILTER.max));
      }}>恢复默认值（关闭）</button>
    </div>
    <small role="status">
      {previewError || error || (bounds == undefined ? "请输入有效数值；右边界 y 必须小于左边界 y。" : kernelMode
        ? "Kernel Mode 不使用 Planner 过滤。" : !available
          ? "本地预览无需 HDU；连接 Planner 后才能应用。" : dirty
            ? "左侧已预览填写范围；尚未应用到 Planner。"
            : `Planner 已确认 · ${state?.persistent === true ? "设置已保存" : "当前会话"}`)}
    </small>
    <small>本地预览：{!enabled ? "隐藏" : bounds == undefined || kernelMode ? "不可用" : `左 ${bounds.max} / 右 ${bounds.min} m`} · 向右 y 变小</small>
    {state != undefined && <small>Planner 当前生效：{state.enabled ? `左 ${state.max.toFixed(5)} / 右 ${state.min.toFixed(5)} m` : "关闭"}</small>}
  </div>;
}
