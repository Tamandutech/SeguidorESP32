#pragma once

#include "tasks/cli/wire_protocol.hpp"

namespace cli_param {

using WireCommand = wire::Command;

enum class ParseError {
  SUCCESS = 0,
  EMPTY_STRING,
  NO_DOT,
  MULTIPLE_DOTS,
  EMPTY_CLASS_NAME,
  EMPTY_PARAMETER_NAME,
  INVALID_FORMAT
};

struct ParsedReference {
  char className[64];
  char parameterName[64];
};

ParseError parseClassNameParameter(const char *input, ParsedReference &result);

bool parseCliFloat(const char *value, float *out);

bool getParameterValue(const char *className, const char *parameterName,
                       char *valueBuffer, size_t bufferSize);

bool setParameterValue(const char *className, const char *parameterName,
                       const char *value);

bool paramSetPersistWireError();

int paramSetRamOnly(const char *refWire, const char *valueWire);

int wireParamList();
int wireParamGet(const WireCommand &w);
int wireParamSetSingle(const WireCommand &w);
int wireParamSetBodyRamOnly(const WireCommand &w);
int wireParamSetLoneBody(const WireCommand &w);

} // namespace cli_param
