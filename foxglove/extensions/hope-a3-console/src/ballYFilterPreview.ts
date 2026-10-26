import { DEFAULT_BALL_Y_FILTER, parseBallYBounds } from "./ballYFilter";

export type BallYFilterDraft = { enabled: boolean; min: string; max: string };
export const PREVIEW_LAYER = "hope-local-y-filter";

// The first preview release saved its automatic default as an edited draft.
// Migrate only that exact legacy tuple; keep genuine edits (including invalid ones).
function isLegacyDefault(value: unknown): boolean {
  const saved = value as { ballYFilterDraft?: BallYFilterDraft; ballYFilterDraftEdited?: boolean } | undefined;
  const draft = saved?.ballYFilterDraft;
  return saved?.ballYFilterDraftEdited == undefined && draft?.enabled === true &&
    Number(draft.min) === -1.38959 && Number(draft.max) === -0.554359;
}

export function ballYFilterDraftEdited(value: unknown): boolean {
  const saved = value as { ballYFilterDraft?: unknown; ballYFilterDraftEdited?: boolean } | undefined;
  return !isLegacyDefault(value) && (saved?.ballYFilterDraftEdited ?? saved?.ballYFilterDraft != undefined);
}

export function readBallYFilterDraft(value: unknown): BallYFilterDraft {
  const draft = (value as { ballYFilterDraft?: BallYFilterDraft } | undefined)?.ballYFilterDraft;
  if (!isLegacyDefault(value) && draft != undefined && typeof draft.enabled === "boolean" &&
      typeof draft.min === "string" && typeof draft.max === "string") { return draft; }
  return { enabled: DEFAULT_BALL_Y_FILTER.enabled,
    min: String(Number(DEFAULT_BALL_Y_FILTER.min.toFixed(6))),
    max: String(Number(DEFAULT_BALL_Y_FILTER.max.toFixed(6))) };
}

/** Local visual geometry only: never publishes ROS messages or changes Planner settings. */
export function ballYPreviewUrdf(min: number, max: number): string {
  const box = (name: string, y: number, z: number, width: number, height: number, rgba: string) =>
    `<link name="${name}"><visual><geometry><box size="2.74 ${width} ${height}"/></geometry>` +
    `<material name="${name}_color"><color rgba="${rgba}"/></material></visual></link>` +
    `<joint name="world_to_${name}" type="fixed"><parent link="world"/><child link="${name}"/>` +
    `<origin xyz="1.37 ${y} ${z}" rpy="0 0 0"/></joint>`;
  const lo = Math.max(-1.525, min), hi = Math.min(0, max);
  return `<?xml version="1.0"?><robot name="hope_local_y_filter_preview"><link name="world"/>` +
    box("hope_preview_left", max, 0.025, 0.025, 0.025, "1 0.65 0.12 1") +
    box("hope_preview_right", min, 0.025, 0.025, 0.025, "1 0.65 0.12 1") +
    (hi > lo ? box("hope_preview_lane", (lo + hi) / 2, 0.006, hi - lo, 0.004, "0.15 0.9 0.55 0.16") : "") +
    `</robot>`;
}

/** Merge into the existing built-in 3D panel; preserve camera, table, ball and pelvis. */
export function withBallYPreview(existing: unknown, draft: BallYFilterDraft, options: { kernelMode: boolean }): Record<string, unknown> {
  const config = (existing ?? {}) as Record<string, unknown>;
  const layers = { ...(config.layers as Record<string, unknown> | undefined) };
  const bounds = parseBallYBounds(draft.min, draft.max);
  const visible = draft.enabled && bounds != undefined && !options.kernelMode;
  layers[PREVIEW_LAYER] = {
    instanceId: PREVIEW_LAYER, layerId: "foxglove.Urdf", visible,
    label: bounds == undefined ? "本地范围预览 · 数值无效" :
      `本地预览 · 左 ${bounds.max} / 右 ${bounds.min} m · 非生效回执`,
    sourceType: "url", framePrefix: "", displayMode: "visual", order: 3,
    url: "data:text/xml;charset=utf-8," + encodeURIComponent(ballYPreviewUrdf(bounds?.min ?? DEFAULT_BALL_Y_FILTER.min, bounds?.max ?? DEFAULT_BALL_Y_FILTER.max)),
  };
  return { ...config, followTf: config.followTf ?? "world", layers,
    // Avoid overlaying stale HDU bars over an editable local preview.
    topics: { ...(config.topics as Record<string, unknown> | undefined),
      "/hope/planner/y_filter_markers": { visible: false } },
  };
}
