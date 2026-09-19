/*
 * Arduino entry point.
 *
 * The Arduino library resolver matches an #include against headers at the top
 * level of src/, so a sketch cannot reach src/sensorhub/sensorhub.h directly
 * even though that directory is on the include path. This header is the
 * conventional shim that makes the library discoverable.
 *
 * In a sketch:      #include <SensorHub.h>
 * Everywhere else:  #include <sensorhub/sensorhub.h>
 *
 * Both reach the same declarations.
 */

#ifndef SENSORHUB_ARDUINO_H
#define SENSORHUB_ARDUINO_H

#include "sensorhub/sensorhub.h"

#endif /* SENSORHUB_ARDUINO_H */
