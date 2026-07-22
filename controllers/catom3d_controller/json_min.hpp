// =============================================================================
// json_min.hpp — minimal dependency-free JSON parser.
//
// Webots controllers build with the bundled MinGW toolchain and no package
// manager, so no third-party JSON library is available. This covers exactly
// what the scene configuration needs: objects, arrays, numbers, strings, bools
// and null, with dotted-path lookup ("world.floor.size").
//
// Extension beyond strict JSON: // and /* */ comments are accepted, so a
// hand-edited config can be annotated. Files we ship stay comment-free and
// therefore remain valid strict JSON for any other tool.
// =============================================================================
#ifndef JSON_MIN_HPP
#define JSON_MIN_HPP

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>

namespace jsonm {

enum class Type { Null, Bool, Number, String, Object, Array };

struct Value {
    Type        type = Type::Null;
    bool        b    = false;
    double      num  = 0.0;
    std::string str;
    std::map<std::string, Value> obj;
    std::vector<Value>           arr;

    bool isNull()   const { return type == Type::Null;   }
    bool isBool()   const { return type == Type::Bool;   }
    bool isNumber() const { return type == Type::Number; }
    bool isString() const { return type == Type::String; }
    bool isObject() const { return type == Type::Object; }
    bool isArray()  const { return type == Type::Array;  }

    // Direct member; nullptr if absent or if this is not an object.
    const Value* member(const std::string& k) const {
        if (type != Type::Object) return nullptr;
        std::map<std::string, Value>::const_iterator it = obj.find(k);
        return it == obj.end() ? nullptr : &it->second;
    }

    // Dotted path: at("world.floor.size"). nullptr if any step is missing.
    const Value* at(const std::string& path) const {
        const Value* cur = this;
        size_t i = 0;
        while (cur) {
            size_t dot = path.find('.', i);
            std::string key = (dot == std::string::npos) ? path.substr(i)
                                                         : path.substr(i, dot - i);
            cur = cur->member(key);
            if (dot == std::string::npos) return cur;
            i = dot + 1;
        }
        return nullptr;
    }
};

struct Result {
    bool        ok = false;
    std::string error;   // "line N: message"
    Value       value;
};

// ---------------------------------------------------------------- parser ----
class Parser {
public:
    explicit Parser(const std::string& s) : s_(s) {}

    Result run() {
        Result r;
        skip();
        Value v;
        if (!parseValue(v)) { r.error = err_; return r; }
        skip();
        if (p_ != s_.size()) { fail("trailing content after top-level value"); r.error = err_; return r; }
        r.ok = true;
        r.value = v;
        return r;
    }

private:
    const std::string& s_;
    size_t      p_ = 0;
    std::string err_;

    int lineAt(size_t pos) const {
        int line = 1;
        for (size_t i = 0; i < pos && i < s_.size(); ++i) if (s_[i] == '\n') ++line;
        return line;
    }
    bool fail(const std::string& msg) {
        if (err_.empty()) {
            std::ostringstream o;
            o << "line " << lineAt(p_) << ": " << msg;
            err_ = o.str();
        }
        return false;
    }
    bool eof() const { return p_ >= s_.size(); }
    char peek() const { return eof() ? '\0' : s_[p_]; }

    // Whitespace plus the comment extension.
    void skip() {
        for (;;) {
            while (!eof() && (s_[p_]==' '||s_[p_]=='\t'||s_[p_]=='\n'||s_[p_]=='\r')) ++p_;
            if (p_ + 1 < s_.size() && s_[p_]=='/' && s_[p_+1]=='/') {
                p_ += 2;
                while (!eof() && s_[p_] != '\n') ++p_;
                continue;
            }
            if (p_ + 1 < s_.size() && s_[p_]=='/' && s_[p_+1]=='*') {
                p_ += 2;
                while (p_ + 1 < s_.size() && !(s_[p_]=='*' && s_[p_+1]=='/')) ++p_;
                p_ = (p_ + 1 < s_.size()) ? p_ + 2 : s_.size();
                continue;
            }
            return;
        }
    }

    bool literal(const char* lit) {
        size_t n = std::string(lit).size();
        if (s_.compare(p_, n, lit) != 0) return false;
        p_ += n;
        return true;
    }

    // Encode one Unicode code point as UTF-8.
    static void utf8(unsigned cp, std::string& out) {
        if (cp < 0x80) out += (char)cp;
        else if (cp < 0x800) {
            out += (char)(0xC0 | (cp >> 6));
            out += (char)(0x80 | (cp & 0x3F));
        } else {
            out += (char)(0xE0 | (cp >> 12));
            out += (char)(0x80 | ((cp >> 6) & 0x3F));
            out += (char)(0x80 | (cp & 0x3F));
        }
    }

    bool hex4(unsigned& out) {
        if (p_ + 4 > s_.size()) return fail("truncated \\u escape");
        unsigned v = 0;
        for (int i = 0; i < 4; ++i) {
            char c = s_[p_ + i];
            v <<= 4;
            if (c >= '0' && c <= '9')      v |= (unsigned)(c - '0');
            else if (c >= 'a' && c <= 'f') v |= (unsigned)(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= (unsigned)(c - 'A' + 10);
            else return fail("bad hex digit in \\u escape");
        }
        p_ += 4;
        out = v;
        return true;
    }

    bool parseString(std::string& out) {
        if (peek() != '"') return fail("expected '\"'");
        ++p_;
        out.clear();
        while (!eof()) {
            char c = s_[p_++];
            if (c == '"') return true;
            if (c != '\\') { out += c; continue; }
            if (eof()) return fail("unterminated escape");
            char e = s_[p_++];
            switch (e) {
                case '"':  out += '"';  break;
                case '\\': out += '\\'; break;
                case '/':  out += '/';  break;
                case 'b':  out += '\b'; break;
                case 'f':  out += '\f'; break;
                case 'n':  out += '\n'; break;
                case 'r':  out += '\r'; break;
                case 't':  out += '\t'; break;
                case 'u': {
                    unsigned cp = 0;
                    if (!hex4(cp)) return false;
                    // Surrogate pair -> single code point.
                    if (cp >= 0xD800 && cp <= 0xDBFF && p_ + 1 < s_.size()
                        && s_[p_] == '\\' && s_[p_+1] == 'u') {
                        p_ += 2;
                        unsigned lo = 0;
                        if (!hex4(lo)) return false;
                        if (lo >= 0xDC00 && lo <= 0xDFFF) {
                            unsigned full = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                            out += (char)(0xF0 | (full >> 18));
                            out += (char)(0x80 | ((full >> 12) & 0x3F));
                            out += (char)(0x80 | ((full >> 6) & 0x3F));
                            out += (char)(0x80 | (full & 0x3F));
                            break;
                        }
                        utf8(cp, out);
                        utf8(lo, out);
                        break;
                    }
                    utf8(cp, out);
                    break;
                }
                default: return fail("unknown escape character");
            }
        }
        return fail("unterminated string");
    }

    bool parseNumber(Value& v) {
        size_t start = p_;
        if (peek() == '-' || peek() == '+') ++p_;
        bool digits = false;
        while (!eof() && s_[p_] >= '0' && s_[p_] <= '9') { ++p_; digits = true; }
        if (!eof() && s_[p_] == '.') {
            ++p_;
            while (!eof() && s_[p_] >= '0' && s_[p_] <= '9') { ++p_; digits = true; }
        }
        if (!digits) return fail("malformed number");
        if (!eof() && (s_[p_] == 'e' || s_[p_] == 'E')) {
            ++p_;
            if (!eof() && (s_[p_] == '-' || s_[p_] == '+')) ++p_;
            bool expDigits = false;
            while (!eof() && s_[p_] >= '0' && s_[p_] <= '9') { ++p_; expDigits = true; }
            if (!expDigits) return fail("malformed exponent");
        }
        v.type = Type::Number;
        v.num  = std::strtod(s_.substr(start, p_ - start).c_str(), nullptr);
        return true;
    }

    bool parseValue(Value& v) {
        skip();
        if (eof()) return fail("unexpected end of input");
        char c = peek();
        if (c == '{') return parseObject(v);
        if (c == '[') return parseArray(v);
        if (c == '"') {
            std::string s;
            if (!parseString(s)) return false;
            v.type = Type::String;
            v.str  = s;
            return true;
        }
        if (literal("true"))  { v.type = Type::Bool; v.b = true;  return true; }
        if (literal("false")) { v.type = Type::Bool; v.b = false; return true; }
        if (literal("null"))  { v.type = Type::Null; return true; }
        if (c == '-' || c == '+' || (c >= '0' && c <= '9')) return parseNumber(v);
        return fail("unexpected character");
    }

    bool parseObject(Value& v) {
        ++p_;                       // consume '{'
        v.type = Type::Object;
        skip();
        if (peek() == '}') { ++p_; return true; }
        for (;;) {
            skip();
            std::string key;
            if (!parseString(key)) return false;
            skip();
            if (peek() != ':') return fail("expected ':' after object key");
            ++p_;
            Value child;
            if (!parseValue(child)) return false;
            v.obj[key] = child;
            skip();
            if (peek() == ',') { ++p_; continue; }
            if (peek() == '}') { ++p_; return true; }
            return fail("expected ',' or '}' in object");
        }
    }

    bool parseArray(Value& v) {
        ++p_;                       // consume '['
        v.type = Type::Array;
        skip();
        if (peek() == ']') { ++p_; return true; }
        for (;;) {
            Value child;
            if (!parseValue(child)) return false;
            v.arr.push_back(child);
            skip();
            if (peek() == ',') { ++p_; continue; }
            if (peek() == ']') { ++p_; return true; }
            return fail("expected ',' or ']' in array");
        }
    }
};

inline Result parse(const std::string& text) { return Parser(text).run(); }

inline Result parseFile(const std::string& path) {
    std::ifstream f(path.c_str());
    if (!f.is_open()) {
        Result r;
        r.error = "cannot open " + path;
        return r;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parse(ss.str());
}

// ------------------------------------------------------- typed accessors ----
// Each returns the default when the path is missing OR holds the wrong type,
// so a partial config file is always valid.

inline double numberOr(const Value& root, const char* path, double dflt) {
    const Value* v = root.at(path);
    return (v && v->isNumber()) ? v->num : dflt;
}
inline bool boolOr(const Value& root, const char* path, bool dflt) {
    const Value* v = root.at(path);
    return (v && v->isBool()) ? v->b : dflt;
}
inline std::string stringOr(const Value& root, const char* path, const std::string& dflt) {
    const Value* v = root.at(path);
    return (v && v->isString()) ? v->str : dflt;
}
// Fills out[0..2] from an array of 3 numbers; leaves it untouched otherwise.
// Returns true when the path supplied a usable vector.
inline bool vec3Into(const Value& root, const char* path, double out[3]) {
    const Value* v = root.at(path);
    if (!v || !v->isArray() || v->arr.size() != 3) return false;
    for (int i = 0; i < 3; ++i) if (!v->arr[i].isNumber()) return false;
    for (int i = 0; i < 3; ++i) out[i] = v->arr[i].num;
    return true;
}

// Every leaf path present in the document ("world.floor.size"), for typo
// detection against a known-key list. Arrays count as leaves.
inline void collectPaths(const Value& v, const std::string& prefix,
                         std::vector<std::string>& out) {
    if (v.isObject()) {
        for (std::map<std::string, Value>::const_iterator it = v.obj.begin();
             it != v.obj.end(); ++it) {
            std::string path = prefix.empty() ? it->first : prefix + "." + it->first;
            if (it->second.isObject() && !it->second.obj.empty())
                collectPaths(it->second, path, out);
            else
                out.push_back(path);
        }
    } else if (!prefix.empty()) {
        out.push_back(prefix);
    }
}

} // namespace jsonm

#endif // JSON_MIN_HPP
