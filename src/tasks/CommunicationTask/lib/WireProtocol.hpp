#ifndef WIRE_PROTOCOL_HPP
#define WIRE_PROTOCOL_HPP

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <strings.h>
#include <vector>

#include "CommunicationUtils.hpp"

// Text wire protocol helpers (see wire_protocol_firmware_implementation.md).
namespace wire {

constexpr int    kMaxArgs     = 24;
constexpr size_t kArgCap      = 160;
constexpr size_t kNameCap     = 64;
constexpr size_t kPackBudget  = 240;

struct Command {
  char name[kNameCap];
  char mode;
  char role;
  int  bodyIndex;
  int  argc;
  char argv[kMaxArgs][kArgCap];
};

inline bool nameEq(const char *a, const char *b) {
  return strcasecmp(a, b) == 0;
}

inline void trimView(const char *s, size_t n, size_t &a, size_t &b) {
  a = 0;
  b = n;
  while(a < b && isspace(static_cast<unsigned char>(s[a]))) {
    a++;
  }
  while(b > a && isspace(static_cast<unsigned char>(s[b - 1]))) {
    b--;
  }
}

inline bool splitTopLevel(const char *s, size_t n, char delim,
                          std::vector<std::pair<size_t, size_t>> &ranges) {
  ranges.clear();
  int     depth = 0;
  bool    inStr = false;
  size_t  start = 0;
  for(size_t i = 0; i <= n; i++) {
    if(i == n || (s[i] == delim && !inStr && depth == 0)) {
      size_t a, b;
      trimView(s + start, i - start, a, b);
      if(a < b) {
        ranges.push_back({start + a, start + b});
      }
      start = i + 1;
      continue;
    }
    char c = s[i];
    if(inStr) {
      if(c == '\\' && i + 1 < n) {
        i++;
        continue;
      }
      if(c == '"') {
        inStr = false;
      }
    } else {
      if(c == '"') {
        inStr = true;
      } else if(c == '(') {
        depth++;
      } else if(c == ')' && depth > 0) {
        depth--;
      }
    }
  }
  return true;
}

inline bool findMatchingClose(const char *s, size_t openIdx, size_t n,
                              size_t &closeIdx) {
  int  depth = 0;
  bool inStr = false;
  for(size_t i = openIdx; i < n; i++) {
    char c = s[i];
    if(inStr) {
      if(c == '\\' && i + 1 < n) {
        i++;
        continue;
      }
      if(c == '"') {
        inStr = false;
      }
    } else {
      if(c == '"') {
        inStr = true;
      } else if(c == '(') {
        depth++;
      } else if(c == ')') {
        depth--;
        if(depth == 0) {
          closeIdx = i;
          return true;
        }
      }
    }
  }
  return false;
}

inline bool unquoteField(const char *src, size_t len, char *out, size_t outCap) {
  if(outCap == 0) {
    return false;
  }
  size_t a, b;
  trimView(src, len, a, b);
  if(a >= b) {
    out[0] = '\0';
    return true;
  }
  src += a;
  len = b - a;
  if(len >= 2 && src[0] == '"' && src[len - 1] == '"') {
    size_t w = 0;
    for(size_t i = 1; i + 1 < len && w + 1 < outCap; i++) {
      if(src[i] == '\\' && i + 1 < len - 1) {
        char nxt = src[i + 1];
        if(nxt == '\\' || nxt == '"') {
          out[w++] = nxt;
          i++;
          continue;
        }
      }
      out[w++] = src[i];
    }
    out[w] = '\0';
    return true;
  }
  if(len >= outCap) {
    return false;
  }
  memcpy(out, src, len);
  out[len] = '\0';
  return true;
}

inline bool copyTokenToArgv(Command &cmd, int idx, const char *src, size_t len) {
  if(idx < 0 || idx >= kMaxArgs) {
    return false;
  }
  return unquoteField(src, len, cmd.argv[idx], kArgCap);
}

inline bool tokenIsPlainInteger(const char *s) {
  if(s == nullptr || s[0] == '\0') {
    return false;
  }
  const char *p = s;
  if(*p == '-') {
    p++;
  }
  if(*p == '\0') {
    return false;
  }
  for(; *p; p++) {
    if(!isdigit(static_cast<unsigned char>(*p))) {
      return false;
    }
  }
  return true;
}

inline bool tokenIsPlainIdentifier(const char *s) {
  if(s == nullptr || s[0] == '\0') {
    return false;
  }
  if(!isalpha(static_cast<unsigned char>(*s)) && *s != '_') {
    return false;
  }
  for(const char *p = s + 1; *p; p++) {
    if(!isalnum(static_cast<unsigned char>(*p)) && *p != '_') {
      return false;
    }
  }
  return true;
}

inline bool tokenNeedsQuotes(const char *s) {
  return !tokenIsPlainInteger(s) && !tokenIsPlainIdentifier(s);
}

inline bool appendWireToken(char *out, size_t outCap, size_t &pos,
                            const char *s) {
  auto appendRaw = [&](const char *r, size_t n) -> bool {
    if(pos + n >= outCap) {
      return false;
    }
    memcpy(out + pos, r, n);
    pos += n;
    out[pos] = '\0';
    return true;
  };

  if(!tokenNeedsQuotes(s)) {
    size_t n = strlen(s);
    return appendRaw(s, n);
  }
  if(!appendRaw("\"", 1)) {
    return false;
  }
  for(const char *p = s; *p; p++) {
    if(*p == '\\' || *p == '"') {
      if(!appendRaw("\\", 1) || !appendRaw(p, 1)) {
        return false;
      }
    } else {
      if(!appendRaw(p, 1)) {
        return false;
      }
    }
  }
  return appendRaw("\"", 1);
}

inline void emitLineUtf8(const char *wireLine) {
  pushWireLineToQueue("%s", wireLine);
}

inline bool appendComma(char *buf, size_t cap, size_t &pos) {
  if(pos + 2 >= cap) {
    return false;
  }
  buf[pos++] = ',';
  buf[pos]   = '\0';
  return true;
}

inline bool emitSingleResponse(const char *cmdName,
                               const std::vector<const char *> &parts) {
  char   buf[MESSAGE_LOG_MESSAGE_SIZE];
  size_t pos = 0;
  int    nw  = snprintf(buf, sizeof(buf), "%s(s,s", cmdName);
  if(nw < 0 || static_cast<size_t>(nw) >= sizeof(buf)) {
    return false;
  }
  pos = static_cast<size_t>(nw);
  for(const char *p : parts) {
    if(!appendComma(buf, sizeof(buf), pos)) {
      return false;
    }
    if(!appendWireToken(buf, sizeof(buf), pos, p)) {
      return false;
    }
  }
  if(pos + 2 >= sizeof(buf)) {
    return false;
  }
  buf[pos++] = ')';
  buf[pos++] = ';';
  buf[pos]   = '\0';
  emitLineUtf8(buf);
  return true;
}

inline void emitBatchAck(const char *cmdName, int messageIndex) {
  char lo[16];
  char hi[16];
  snprintf(lo, sizeof(lo), "%d", messageIndex);
  snprintf(hi, sizeof(hi), "%d", messageIndex);
  emitSingleResponse(cmdName, {lo, hi, "ok"});
}

inline bool parseSegment(const char *seg, size_t segLen, Command &out) {
  memset(&out, 0, sizeof(out));
  out.bodyIndex = -1;
  char lineBuf[512];
  if(segLen >= sizeof(lineBuf)) {
    return false;
  }
  memcpy(lineBuf, seg, segLen);
  lineBuf[segLen] = '\0';

  size_t n = strlen(lineBuf);
  size_t openIdx = n;
  for(size_t i = 0; i < n; i++) {
    if(lineBuf[i] == '(') {
      openIdx = i;
      break;
    }
  }
  if(openIdx >= n) {
    return false;
  }
  size_t nameA, nameB;
  trimView(lineBuf, openIdx, nameA, nameB);
  if(nameA >= nameB || nameB - nameA >= kNameCap) {
    return false;
  }
  memcpy(out.name, lineBuf + nameA, nameB - nameA);
  out.name[nameB - nameA] = '\0';

  size_t closeIdx = 0;
  if(!findMatchingClose(lineBuf, openIdx, n, closeIdx)) {
    return false;
  }
  const char *inner    = lineBuf + openIdx + 1;
  size_t      innerLen = closeIdx - openIdx - 1;

  std::vector<std::pair<size_t, size_t>> commaRanges;
  splitTopLevel(inner, innerLen, ',', commaRanges);
  if(commaRanges.size() < 2) {
    return false;
  }
  out.argc = static_cast<int>(commaRanges.size());
  if(out.argc > kMaxArgs) {
    return false;
  }
  for(int i = 0; i < out.argc; i++) {
    auto pr = commaRanges[static_cast<size_t>(i)];
    if(!copyTokenToArgv(out, i, inner + pr.first, pr.second - pr.first)) {
      return false;
    }
  }
  if(strlen(out.argv[0]) != 1 || strlen(out.argv[1]) != 1) {
    return false;
  }
  out.mode = out.argv[0][0];
  out.role = out.argv[1][0];
  if(out.mode == 'b') {
    if(out.argc < 3) {
      return false;
    }
    out.bodyIndex = atoi(out.argv[2]);
  }
  return true;
}

inline bool appendListBody(char *buf, size_t cap, size_t &pos,
                           const char *cmdName, char listMode, char listRole,
                           int idx, const std::vector<const char *> &fields) {
  int nw = snprintf(buf + pos, cap - pos, "%s(%c,%c,%d", cmdName, listMode,
                    listRole, idx);
  if(nw < 0 || static_cast<size_t>(nw) >= cap - pos) {
    return false;
  }
  pos += static_cast<size_t>(nw);
  for(const char *f : fields) {
    if(!appendComma(buf, cap, pos)) {
      return false;
    }
    if(!appendWireToken(buf, cap, pos, f)) {
      return false;
    }
  }
  if(pos + 2 >= cap) {
    return false;
  }
  buf[pos++] = ')';
  buf[pos++] = ';';
  buf[pos]   = '\0';
  return true;
}

inline bool appendListHeader(char *buf, size_t cap, size_t &pos,
                             const char *cmdName, int T, int C, int B, int j) {
  char ts[16], cs[16], bs[16], js[16];
  snprintf(ts, sizeof(ts), "%d", T);
  snprintf(cs, sizeof(cs), "%d", C);
  snprintf(bs, sizeof(bs), "%d", B);
  snprintf(js, sizeof(js), "%d", j);
  int nw = snprintf(buf + pos, cap - pos, "%s(h,s,%s,%s,%s,%s);", cmdName, ts,
                    cs, bs, js);
  if(nw < 0 || static_cast<size_t>(nw) >= cap - pos) {
    return false;
  }
  pos += static_cast<size_t>(nw);
  return true;
}

inline void flushListChunk(const char *cmdName,
                           const std::vector<std::string> &bodySegments, int T,
                           int B, int j) {
  char   buf[MESSAGE_LOG_MESSAGE_SIZE];
  size_t pos = 0;
  int    C   = static_cast<int>(bodySegments.size());
  if(!appendListHeader(buf, sizeof(buf), pos, cmdName, T, C, B, j)) {
    return;
  }
  for(const std::string &seg : bodySegments) {
    size_t len = seg.size();
    if(pos + len + 1 >= sizeof(buf)) {
      return;
    }
    memcpy(buf + pos, seg.c_str(), len);
    pos += len;
  }
  buf[pos] = '\0';
  emitLineUtf8(buf);
}

// Wire length of `name(h,s,T,C,B,j);` (matches appendListHeader).
inline size_t listHeaderWireBytes(const char *cmdName, int T, int C, int B,
                                  int j) {
  char tmp[160];
  int  n = snprintf(tmp, sizeof(tmp), "%s(h,s,%d,%d,%d,%d);", cmdName, T, C, B,
                    j);
  if(n < 0) {
    return sizeof(tmp);
  }
  return static_cast<size_t>(n);
}

// Each element must already be a full `name(b,s,idx,...);` segment (trailing
// `;` is part of the command). Pack into BLE lines: if the next body does not
// fit after the list header for this chunk (pessimistic T,C,B,j per host
// packing), start a new chunk — that chunk begins with a fresh `h,s` header.
inline void emitListFromBodySegments(
    const char *cmdName, const std::vector<std::string> &bodies) {
  int T = static_cast<int>(bodies.size());
  if(T == 0) {
    flushListChunk(cmdName, {}, 0, 1, 0);
    return;
  }
  const int B_pess = std::max(1, T);
  const int j_pess = std::max(0, T - 1);

  std::vector<std::vector<std::string>> chunks;
  std::vector<std::string>               cur;
  size_t                                 sumBodies = 0;

  for(const std::string &b : bodies) {
    const int nextC = static_cast<int>(cur.size()) + 1;
    const size_t hdrBytes =
        listHeaderWireBytes(cmdName, T, nextC, B_pess, j_pess);
    const size_t totalIfAdd = hdrBytes + sumBodies + b.size();
    if(!cur.empty() && totalIfAdd > kPackBudget) {
      chunks.push_back(std::move(cur));
      cur.clear();
      sumBodies = 0;
    }
    cur.push_back(b);
    sumBodies += b.size();
  }
  if(!cur.empty()) {
    chunks.push_back(std::move(cur));
  }

  int B = static_cast<int>(chunks.size());
  if(B < 1) {
    B = 1;
  }
  for(int j = 0; j < static_cast<int>(chunks.size()); j++) {
    flushListChunk(cmdName, chunks[static_cast<size_t>(j)], T, B, j);
  }
}

inline std::string commandKeyLower(const char *name) {
  std::string k(name);
  for(char &c : k) {
    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
  }
  return k;
}

} // namespace wire

#endif // WIRE_PROTOCOL_HPP
