// tools/boundt_to_json_ms.cpp
// Convert Bound-T report -> JSON with mangled & demangled names
// Usage: ./boundt_to_json_ms <boundt_report.txt> <F_CPU_Hz> <out.json>
//        Use "-" as <out.json> to write to stdout.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(__GNUG__)
  #include <cxxabi.h>
#endif

// ----------------- helpers -----------------
static uint64_t parse_fcpu(const std::string& s) {
    // Accept "16000000" or "16000000UL"
    uint64_t v = 0;
    for (char c : s) if (c >= '0' && c <= '9') v = v*10 + (c - '0');
    return v;
}

static uint64_t ceil_cycles_to_us(uint64_t cycles, uint64_t fcpu_hz) {
    if (fcpu_hz == 0) return 0;
    // ceil(cycles * 1e6 / F_CPU)
    return (cycles * 1000000ULL + fcpu_hz - 1ULL) / fcpu_hz;
}

static double cycles_to_ms_exact(uint64_t cycles, uint64_t fcpu_hz) {
    return (fcpu_hz == 0) ? 0.0
                          : (static_cast<double>(cycles) * 1000.0 / static_cast<double>(fcpu_hz));
}

static uint64_t round_ms_nearest(double ms) {
    // nearest integer (0.5 -> up)
    return (ms < 0.0) ? 0ULL : static_cast<uint64_t>(ms + 0.5);
}

// Simple JSON string escaper
static std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '\"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[7];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back(static_cast<char>(c));
                }
        }
    }
    return out;
}

// Try to demangle Itanium C++ names; returns input if not demangle-able
static std::string demangle(const std::string& mangled) {
#if defined(__GNUG__)
    int status = 0;
    std::size_t len = 0;
    char* p = abi::__cxa_demangle(mangled.c_str(), nullptr, &len, &status);
    if (status == 0 && p) {
        std::string out(p);
        std::free(p);
        return out;
    }
#endif
    return mangled; // fallback: unchanged
}

// ----------------- extra data structures -----------------
struct WcetRec {
    std::string sym, dem, range;
    uint64_t cycles = 0, us_ceil = 0, ms_int = 0;
    double ms_exact = 0.0;
};
struct WcetCall {
    std::string caller, caller_dem, caller_site;
    std::string callee, callee_dem, callee_range;
    uint64_t cycles = 0;
};
struct LoopBound {
    std::string context, callee, callee_range;
    uint64_t max_iter = 0;
    bool unreachable = false;
};
struct TimeRow {
    std::string func, func_range, symbol, symbol_dem, symbol_range;
    std::vector<uint64_t> nums;
};

// ----------------- main -----------------
int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: " << argv[0] << " <boundt_report.txt> <F_CPU_Hz> <out.json>\n";
        std::cerr << "hint : F_CPU_Hz e.g. 16000000 or 16000000UL\n";
        return 2;
    }

    const std::string report_path = argv[1];
    const uint64_t F_CPU = parse_fcpu(argv[2]);
    const std::string out_path = argv[3];

    std::ifstream in(report_path);
    if (!in) {
        std::cerr << "error: cannot open report file: " << report_path << "\n";
        return 1;
    }

    // Keep max cycles per mangled symbol found in WCET/Time_Table lines
    std::unordered_map<std::string, uint64_t> wcet_cycles;

    // Structured outputs
    std::vector<WcetRec>  wcet_vec;
    std::vector<WcetCall> wcet_calls;
    std::vector<LoopBound> loop_bounds;
    std::vector<TimeRow>  time_rows;
    std::vector<std::string> warnings;

    // Regexes
    std::regex line_ok(R"(^\s*(Wcet:|Time_Table:).*)");
    // Looser Itanium matcher: starts with _Z and consumes typical chars (also allow '@' suffix removal)
    std::regex sym_rgx(R"((_Z[A-Za-z0-9_]+))");
    std::regex tail_num(R"((\d+)\s*$)"); // cycles at end of line

    // Structured parsers
    std::regex wcet_line   (R"(^\s*Wcet:.*::([^\[]]+):(\[[0-9A-Fa-f\-]+\]):(\d+)\s*$)");
    std::regex wcet_call   (R"(^\s*Wcet_Call:.*::([^\[]@?\[[0-9A-Fa-f]+\])=>([^:]+):(\[[0-9A-Fa-f\-]+\]):(\d+)\s*$)");
    std::regex loop_line   (R"(^\s*Loop_Bound:.*::([^@]+)@(\[[0-9A-Fa-f]+\])=>([^:]+):(\[[0-9A-Fa-f\-]+\]):(\d+)\s*$)");
    std::regex warn_line   (R"(^\s*Warning:.*$)");
    std::regex time_table  (R"(^\s*Time_Table:.*::([^\[]]+):(\[[0-9A-Fa-f\-]+\]):((?:\d+:)+)([^:]+)::(\[[0-9A-Fa-f\-]+\])\s*$)");

    // helper: split "a:b:c:" → vector<uint64_t>
    auto split_nums = [](const std::string& s){
        std::vector<uint64_t> v;
        std::string tok;
        std::stringstream ss(s);
        while (std::getline(ss, tok, ':')) {
            if (!tok.empty()) v.push_back(std::strtoull(tok.c_str(), nullptr, 10));
        }
        return v;
    };

    std::string line;
    while (std::getline(in, line)) {

        // 1) Structured matches (populate extended JSON)
        std::smatch m;

        // Wcet (per function)
        if (std::regex_match(line, m, wcet_line)) {
            std::string sym = m[1];
            if (auto at = sym.find('@'); at != std::string::npos) sym.erase(at);
            WcetRec r;
            // trim trailing spaces around symbol, if any
            while (!sym.empty() && (sym.back()==' ' || sym.back()=='\t')) sym.pop_back();
            r.sym   = sym;
            r.dem   = demangle(sym);
            r.range = m[2];
            r.cycles   = std::strtoull(m[3].str().c_str(), nullptr, 10);
            r.ms_exact = cycles_to_ms_exact(r.cycles, F_CPU);
            r.ms_int   = round_ms_nearest(r.ms_exact);
            r.us_ceil  = ceil_cycles_to_us(r.cycles, F_CPU);
            wcet_vec.push_back(std::move(r));
            // fall-through to generic per-symbol collector is not necessary here
            continue;
        }

        // Wcet_Call
        if (std::regex_match(line, m, wcet_call)) {
            WcetCall c;
            c.caller = m[1].str();
            if (auto at1 = c.caller.find('@'); at1 != std::string::npos) {
                c.caller_site = c.caller.substr(at1+1);
                c.caller.erase(at1);
            }
            c.caller_dem = demangle(c.caller);

            c.callee = m[2].str();
            if (auto at2 = c.callee.find('@'); at2 != std::string::npos) c.callee.erase(at2);
            c.callee_dem   = demangle(c.callee);
            c.callee_range = m[3];
            c.cycles = std::strtoull(m[4].str().c_str(), nullptr, 10);
            wcet_calls.push_back(std::move(c));
            // do not continue to generic collector, WCET_Call lines end with a number that is not per-symbol max
            continue;
        }

        // Loop_Bound
        if (std::regex_match(line, m, loop_line)) {
            LoopBound lb;
            lb.context = m[1].str() + "@" + m[2].str(); // keep original anchor
            lb.callee  = m[3].str();
            if (auto at = lb.callee.find('@'); at != std::string::npos) lb.callee.erase(at);
            lb.callee_range = m[4];
            lb.max_iter = std::strtoull(m[5].str().c_str(), nullptr, 10);
            loop_bounds.push_back(std::move(lb));
            continue;
        }

        // Warning
        if (std::regex_match(line, warn_line)) {
            warnings.push_back(line);
            continue;
        }

        // Time_Table
        if (std::regex_match(line, m, time_table)) {
            TimeRow t;
            t.func = m[1];
            // trim spaces which sometimes appear around names
            t.func.erase(std::remove(t.func.begin(), t.func.end(), ' '), t.func.end());
            t.func_range = m[2];
            t.nums       = split_nums(m[3]);
            t.symbol     = m[4];
            // strip any trailing spaces
            while (!t.symbol.empty() && (t.symbol.back()==' ' || t.symbol.back()=='\t')) t.symbol.pop_back();
            t.symbol_dem = demangle(t.symbol);
            t.symbol_range = m[5];
            time_rows.push_back(std::move(t));
            // also consider for generic per-symbol max (if symbol is mangled and a number belongs to row)
            // but we already have better structured data; the generic collector below will also see Time_Table.
            // Don't continue; allow generic collector to potentially update per-symbol maxima.
        }

        // 2) Generic per-symbol MAX cycles collector (as in original)
        if (!std::regex_search(line, line_ok)) continue;

        // Use the last mangled symbol on the line (most specific context)
        std::sregex_iterator it(line.begin(), line.end(), sym_rgx), end;
        if (it == end) continue;
        std::string sym;
        for (; it != end; ++it) sym = it->str(); // last match wins

        // Some tools append @plt / @thumb etc.; strip after '@' if present
        if (auto pos = sym.find('@'); pos != std::string::npos) sym.erase(pos);

        std::smatch mnum;
        if (!std::regex_search(line, mnum, tail_num)) continue;

        uint64_t cyc = 0;
        try { cyc = std::stoull(mnum.str(1)); }
        catch (...) { continue; }

        auto& slot = wcet_cycles[sym];
        if (cyc > slot) slot = cyc; // keep the maximum per symbol
    }

    // Emit JSON
    std::ostringstream json;
    json << "{\n";
    json << "  \"F_CPU_Hz\": " << F_CPU << ",\n";

    // Emit compact summary records (backwards-compat)
    json << "  \"records\": [\n";

    // Stable order
    std::vector<std::pair<std::string,uint64_t>> items(wcet_cycles.begin(), wcet_cycles.end());
    std::sort(items.begin(), items.end(), [](auto& a, auto& b){ return a.first < b.first; });

    for (size_t i = 0; i < items.size(); ++i) {
        const std::string& mangled = items[i].first;
        const uint64_t cyc         = items[i].second;

        const double   ms_exact = cycles_to_ms_exact(cyc, F_CPU);
        const uint64_t ms_int   = round_ms_nearest(ms_exact);
        const uint64_t us_ceil  = ceil_cycles_to_us(cyc, F_CPU);

        const std::string dem = demangle(mangled);

        json << "    {"
             << "\"mangled\": \""   << json_escape(mangled) << "\", "
             << "\"demangled\": \"" << json_escape(dem)     << "\", "
             << "\"cycles\": "      << cyc                  << ", "
             << "\"wcet_us_ceil\": " << us_ceil             << ", "
             << "\"wcet_ms_exact\": " << std::fixed << std::setprecision(6) << ms_exact << ", "
             << "\"wcet_ms\": "      << ms_int
             << "}";

        if (i + 1 < items.size()) json << ",";
        json << "\n";

        // reset flags (precision sticks on the stream)
        json.unsetf(std::ios::floatfield);
        json << std::setprecision(6);
    }
    json << "  ],\n";

    // Extended sections
    // wcet (structured)
    json << "  \"wcet\": [\n";
    for (size_t i=0;i<wcet_vec.size();++i){
        const auto& r = wcet_vec[i];
        json << "    {\"symbol\":\"" << json_escape(r.sym) << "\", "
             << "\"demangled\":\"" << json_escape(r.dem) << "\", "
             << "\"addr_range\":\"" << json_escape(r.range) << "\", "
             << "\"cycles\":" << r.cycles << ", "
             << "\"wcet_us_ceil\":" << r.us_ceil << ", "
             << "\"wcet_ms_exact\":" << std::fixed << std::setprecision(6) << r.ms_exact << ", "
             << "\"wcet_ms\":" << r.ms_int << "}";
        if (i+1<wcet_vec.size()) json << ",";
        json << "\n";
        json.unsetf(std::ios::floatfield);
        json << std::setprecision(6);
    }
    json << "  ],\n";

    // wcet_calls
    json << "  \"wcet_calls\": [\n";
    for (size_t i=0;i<wcet_calls.size();++i){
        const auto& c = wcet_calls[i];
        json << "    {\"caller\":\"" << json_escape(c.caller) << "\", "
             << "\"caller_dem\":\"" << json_escape(c.caller_dem) << "\", "
             << "\"caller_site\":\"" << json_escape(c.caller_site) << "\", "
             << "\"callee\":\"" << json_escape(c.callee) << "\", "
             << "\"callee_dem\":\"" << json_escape(c.callee_dem) << "\", "
             << "\"callee_range\":\"" << json_escape(c.callee_range) << "\", "
             << "\"cycles\":" << c.cycles << "}";
        if (i+1<wcet_calls.size()) json << ",";
        json << "\n";
    }
    json << "  ],\n";

    // loop_bounds
    json << "  \"loop_bounds\": [\n";
    for (size_t i=0;i<loop_bounds.size();++i){
        const auto& lb = loop_bounds[i];
        json << "    {\"context\":\"" << json_escape(lb.context) << "\", "
             << "\"callee\":\"" << json_escape(lb.callee) << "\", "
             << "\"callee_range\":\"" << json_escape(lb.callee_range) << "\", "
             << "\"max_iterations\":" << lb.max_iter << "}";
        if (i+1<loop_bounds.size()) json << ",";
        json << "\n";
    }
    json << "  ],\n";

    // time_table
    json << "  \"time_table\": [\n";
    for (size_t i=0;i<time_rows.size();++i){
        const auto& t = time_rows[i];
        json << "    {\"function\":\"" << json_escape(t.func) << "\", "
             << "\"func_range\":\"" << json_escape(t.func_range) << "\", "
             << "\"nums\":[";
        for (size_t j=0;j<t.nums.size();++j){
            json << t.nums[j];
            if (j+1<t.nums.size()) json << ",";
        }
        json << "], "
             << "\"symbol\":\"" << json_escape(t.symbol) << "\", "
             << "\"symbol_dem\":\"" << json_escape(t.symbol_dem) << "\", "
             << "\"symbol_range\":\"" << json_escape(t.symbol_range) << "\"}";
        if (i+1<time_rows.size()) json << ",";
        json << "\n";
    }
    json << "  ],\n";

    // warnings
    json << "  \"warnings\": [\n";
    for (size_t i=0;i<warnings.size();++i){
        json << "    \"" << json_escape(warnings[i]) << "\"";
        if (i+1<warnings.size()) json << ",";
        json << "\n";
    }
    json << "  ]\n";

    json << "}\n";

    if (out_path == "-") {
        std::cout << json.str();
    } else {
        std::ofstream out(out_path, std::ios::trunc);
        if (!out) {
            std::cerr << "error: cannot write: " << out_path << "\n";
            return 1;
        }
        out << json.str();
        out.close();
        std::cout << "Wrote " << out_path << " with " << items.size() << " entries.\n";
    }
    return 0;
}
