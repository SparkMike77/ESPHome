#pragma once
// Helpers for front-door.yaml: Home Assistant weather conditions -> label / Material Design icon.
// The icon glyphs must also be listed in the yaml's mdi_weather font.
#include <string>
#include <ArduinoJson.h>
#include "esphome/core/log.h"

// The JSON object a response_template produced, wherever Home Assistant put it: the response
// itself, nested under "response", or delivered as a JSON string. Logs the raw reply.
inline JsonObjectConst front_door_unwrap(JsonObjectConst response, JsonDocument &scratch, const char *what) {
  std::string raw;
  serializeJson(response, raw);
  ESP_LOGI("front_door", "%s raw response: %.300s", what, raw.c_str());
  JsonVariantConst inner = response["response"];
  if (inner.is<JsonObjectConst>()) return inner.as<JsonObjectConst>();
  if (inner.is<const char *>()) {
    if (deserializeJson(scratch, inner.as<const char *>()) == DeserializationError::Ok)
      return scratch.as<JsonObjectConst>();
  }
  return response;
}

inline std::string cond_text(const std::string &c) {
  if (c == "sunny") return "Sunny";
  if (c == "clear-night") return "Clear";
  if (c == "partlycloudy") return "Partly cloudy";
  if (c == "cloudy") return "Cloudy";
  if (c == "fog") return "Fog";
  if (c == "hail") return "Hail";
  if (c == "lightning") return "Thunderstorms";
  if (c == "lightning-rainy") return "Thunderstorms";
  if (c == "pouring") return "Heavy rain";
  if (c == "rainy") return "Rain";
  if (c == "snowy") return "Snow";
  if (c == "snowy-rainy") return "Rain and snow";
  if (c == "windy" || c == "windy-variant") return "Windy";
  if (c == "exceptional") return "Warning";
  return "";
}

inline std::string cond_icon(const std::string &c) {
  if (c == "sunny") return "\U000F0599";
  if (c == "clear-night") return "\U000F0594";
  if (c == "partlycloudy") return "\U000F0595";
  if (c == "cloudy") return "\U000F0590";
  if (c == "fog") return "\U000F0591";
  if (c == "hail") return "\U000F0592";
  if (c == "lightning") return "\U000F0593";
  if (c == "lightning-rainy") return "\U000F067E";
  if (c == "pouring") return "\U000F0596";
  if (c == "rainy") return "\U000F0597";
  if (c == "snowy") return "\U000F0598";
  if (c == "snowy-rainy") return "\U000F067F";
  if (c == "windy") return "\U000F059D";
  if (c == "windy-variant") return "\U000F059D";
  if (c == "exceptional") return "\U000F05D6";
  return "";
}
