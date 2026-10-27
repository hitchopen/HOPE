import { PanelExtensionContext } from "@foxglove/extension";

export const OPERATOR_CHALLENGE = "/hope/runner/operator_challenge_hdu_flat";
export const OPERATOR_ECHO = "/hope/runner/operator_echo_hdu_flat";

// Standard ROS 2 MultiArray definitions, using the names expected by the
// extension publishing API. No command datatype or arbitrary topic is exposed.
// https://docs.foxglove.dev/docs/extensions/extension-api/type-aliases/PanelExtensionContext
const datatypes = new Map([
  ["std_msgs/Float64MultiArray", { definitions: [
    { name: "layout", type: "std_msgs/MultiArrayLayout", isComplex: true },
    { name: "data", type: "float64", isArray: true },
  ] }],
  ["std_msgs/MultiArrayLayout", { definitions: [
    { name: "dim", type: "std_msgs/MultiArrayDimension", isArray: true, isComplex: true },
    { name: "data_offset", type: "uint32" },
  ] }],
  ["std_msgs/MultiArrayDimension", { definitions: [
    { name: "label", type: "string" }, { name: "size", type: "uint32" },
    { name: "stride", type: "uint32" },
  ] }],
]);

export function sessionFingerprint(session: string): number {
  let value = 1469598103934665603n;
  for (const byte of new TextEncoder().encode(session)) {
    value = BigInt.asUintN(64, (value ^ BigInt(byte)) * 1099511628211n);
  }
  const masked = Number(value & ((1n << 52n) - 1n));
  return masked === 0 ? 1 : masked;
}

export class OperatorHeartbeat {
  #last?: number[];
  #advertised = false;
  constructor(private context: PanelExtensionContext) {}

  echo(value: unknown, boot: number | undefined, session: string | undefined): void {
    if (!this.context.advertise || !this.context.publish || !session || boot == undefined) { return; }
    const values: unknown[] | undefined = Array.isArray(value) ? value
      : ArrayBuffer.isView(value) && !(value instanceof DataView)
        ? Array.from(value as unknown as ArrayLike<unknown>) : undefined;
    if (values?.length !== 4 || values[0] !== 1 ||
        !values.every((x): x is number => typeof x === "number" && Number.isSafeInteger(x) && x > 0 && x <= 2**52) ||
        values[1] !== boot || values[2] !== sessionFingerprint(session)) { return; }
    if (this.#last?.[1] === values[1] && values[2] === this.#last[2] && values[3]! <= this.#last[3]!) { return; }
    try {
      if (!this.#advertised) {
        this.context.advertise(OPERATOR_ECHO, "std_msgs/Float64MultiArray", { datatypes });
        this.#advertised = true;
      }
      this.context.publish(OPERATOR_ECHO, { layout: { dim: [], data_offset: 0 }, data: [...values, 1] });
      this.#last = values;
    } catch {
      this.#advertised = false; // reconnect must retry advertisement, not remain stuck forever
    }
  }

  close(): void {
    if (this.#advertised) {
      try { this.context.unadvertise?.(OPERATOR_ECHO); } catch { /* already disconnected */ }
    }
    this.#advertised = false;
  }
}
