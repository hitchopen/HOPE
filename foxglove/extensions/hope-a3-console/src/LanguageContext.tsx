import { createContext, useContext } from "react";

import { defaultTranslator, Translator } from "./i18n";

export const LanguageContext = createContext(defaultTranslator);
export const useI18n = (): Translator => useContext(LanguageContext);
