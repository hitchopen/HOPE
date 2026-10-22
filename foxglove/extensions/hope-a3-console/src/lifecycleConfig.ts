// Display each authoritative field as it arrives, including on older supervisors.
// Missing fields must not erase confirmed values or block unrelated controls.
export function mergeConfirmedConfig<T extends Record<string, string>>(
  draft: T, confirmed: Partial<T>, touched: boolean,
): T {
  if (touched) { return draft; }
  const next = { ...draft };
  for (const key of Object.keys(draft) as (keyof T)[]) {
    const value = confirmed[key];
    if (value !== undefined) { next[key] = value; }
  }
  return next;
}
