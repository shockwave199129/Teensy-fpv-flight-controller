#pragma once

// Simple logging macro – enable with DEBUG definition
#ifdef DEBUG
#define LOG_DEBUG(msg) Serial.println(msg)
#else
#define LOG_DEBUG(msg) do {} while(0)
#endif