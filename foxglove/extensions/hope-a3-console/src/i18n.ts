import { en, MessageKey } from "./messages.en";
import { zh } from "./messages.zh";

export type Locale = "zh" | "en";
export type LocalText = string | Message;
type Parameter = string | number | boolean | null | undefined | Message;
type Placeholders<S extends string> = S extends `${string}{${infer P}}${infer Rest}`
  ? P | Placeholders<Rest> : never;
type Arguments<K extends MessageKey> = [Placeholders<(typeof en)[K]>] extends [never]
  ? [] : [parameters: Record<Placeholders<(typeof en)[K]>, Parameter>];
export type Message = { key: MessageKey; parameters?: Record<string, Parameter> };
export type Translator = {
  <K extends MessageKey>(key: K, ...args: Arguments<K>): string;
  text: (value: LocalText) => string;
};

/** Store descriptors in state so pending results and errors follow the active language. */
export function message<K extends MessageKey>(key: K, ...args: Arguments<K>): Message {
  return { key, parameters: args[0] };
}

export function createTranslator(locale: Locale): Translator {
  const catalog = locale === "en" ? en : zh;
  const format = (value: LocalText): string => {
    if (typeof value === "string") { return value; }
    return catalog[value.key].replace(/\{(\w+)\}/g, (_, name: string) => {
      const parameter = value.parameters?.[name];
      return typeof parameter === "object" && parameter != null
        ? format(parameter) : String(parameter ?? "");
    });
  };
  const t = ((key: MessageKey, parameters?: Record<string, Parameter>) =>
    format({ key, parameters })) as Translator;
  t.text = format;
  return t;
}

export const defaultTranslator = createTranslator("zh");
export function readLocale(state: unknown): Locale {
  return (state as { language?: unknown } | undefined)?.language === "en" ? "en" : "zh";
}

export class LocalizedError extends Error {
  constructor(readonly text: LocalText) {
    super(defaultTranslator.text(text));
    this.name = "LocalizedError";
  }
}

export function errorText(error: unknown): LocalText {
  return error instanceof LocalizedError ? error.text : error instanceof Error ? error.message : String(error);
}
