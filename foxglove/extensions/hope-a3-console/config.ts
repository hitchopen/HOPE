import type { Configuration } from "webpack";

export function webpack(config: Configuration): Configuration {
  config.module?.rules?.push({
    test: /\.woff2$/i,
    type: "asset/inline",
  });
  return config;
}
