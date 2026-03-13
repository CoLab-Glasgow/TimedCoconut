// annotate_wcet_inplace.cpp — emits/REPLACES WCET_AT("..."); in-place (no pragmas).
//
// Build:
//   g++ -std=c++17 annotate_wcet_inplace.cpp -o annotate_wcet
//
// Run:
//   ./annotate_wcet [--backup] [--debug] [--annotate-missing] [--na-as-minus1] wcet.json your_file.cpp
//
// What it does:
//  1) Inserts/replaces WCET_AT(...) ABOVE each matched *definition* (never between signature and '{').
//  2) If --annotate-missing is set: it also inserts WCET_AT("...N/A...") (or -1) for function definitions
//     in the file that have no WCET record and no existing WCET_AT above them.
//
// Notes:
//  - Uses heuristics (brace scan) to distinguish definitions vs declarations.
//  - Works best on .cpp/.h with normal formatting; it is not a full C++ parser.
//
// EDITED BEHAVIOR:
//  - If wcet.json contains multiple records for the same function, we keep ONLY the one with the LOWEST wcet_ms_exact.
//  - The emitted wcet_ms_exact is ROUNDED DOWN (floor) to 6 decimal places before annotation.

#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <optional>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <cmath> // <-- added
#include "../external/nlohmann/json.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

static bool g_debug = false;

// === CONFIG ===
static constexpr const char* kAnnotMacro = "WCET_AT";
static constexpr bool kAnnotNeedsSemicolon = true;
// ==============

struct Rec {
    std::string demangled, mangled;
    long long cycles{}, wcet_us_ceil{};
    double wcet_ms_exact{};
};

static std::string trim(const std::string& s){
    auto b = s.find_first_not_of(" \t\r\n");
    auto e = s.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, e - b + 1);
}

// Extract a stable "function key" from demangled name: qualified name up to '('
static std::string func_key_from_demangled(const std::string& demangled) {
    auto p = demangled.find('(');
    return trim(p == std::string::npos ? demangled : demangled.substr(0, p));
}

// Regex helper: build a regex that matches the qualified name up to '('
static std::regex build_qualified_regex(const std::string& demangled) {
    auto p = demangled.find('(');
    std::string q = trim(p==std::string::npos? demangled : demangled.substr(0,p));

    std::string esc;
    for (char c: q){
        if (std::string(R"(\.^$|()[]{}*+?-)").find(c)!=std::string::npos) esc.push_back('\\');
        esc.push_back(c);
    }

    std::string pat;
    for (size_t i=0;i<esc.size();++i){
        if (i+1<esc.size() && esc[i]==':' && esc[i+1]==':'){ pat += R"(\s*::\s*)"; ++i; }
        else if (esc[i]==' ') pat += R"(\s+)";
        else pat += esc[i];
    }
    pat += R"(\s*\()";
    return std::regex(pat);
}

// Find '{' for definition; return -1 if a declaration ';' is seen first.
// Starts scanning from the signature line.
// Note: scans up to +200 lines to handle multi-line signatures.
static int find_definition_brace(const std::vector<std::string>& lines, int start_idx){
    int paren = 0;
    for (int j = start_idx; j < std::min<int>((int)lines.size(), start_idx + 200); ++j) {
        for (char ch : lines[j]) {
            if (ch == '(') ++paren;
            else if (ch == ')') paren = std::max(0, paren - 1);
        }

        if (paren == 0) {
            auto posBrace = lines[j].find('{');
            auto posSemi  = lines[j].find(';');

            if (posBrace != std::string::npos) return j;  // definition
            if (posSemi  != std::string::npos) return -1; // declaration
        }
    }
    return -1;
}

// Rewind from a matched line to the top of the function signature block.
// This prevents placing WCET_AT below attributes/templates that belong to the signature.
static int rewind_to_signature_start(const std::vector<std::string>& lines, int sig_line){
    int i = sig_line;
    int steps = 0;

    while (i > 0 && steps < 40) {
        std::string prev = trim(lines[i - 1]);

        if (prev.empty()) break;
        if (prev.rfind("//", 0) == 0) break;

        if (prev.find('}') != std::string::npos) break;
        if (prev.find(';') != std::string::npos) break;

        if (prev.rfind("template", 0) == 0) { --i; ++steps; continue; }
        if (prev.rfind("[[", 0) == 0) { --i; ++steps; continue; }
        if (prev.rfind("__attribute__", 0) == 0) { --i; ++steps; continue; }

        break;
    }
    return i;
}

// Remove any existing annotation near the insertion point.
// Here `sig_idx` is the *signature start line* we will insert before.
// We remove old WCET_AT/WCET_ANNOTATE/#pragma directly above it (skipping blanks/comments).
static bool remove_attr_near(std::vector<std::string>& lines, int& sig_idx){
    bool removed = false;

    // 0) Remove preceding WCET_AT("...") / WCET_AT("..."); directly tied to THIS signature only.
    {
        std::regex at_line(std::string(R"(^\s*)") + kAnnotMacro +
                           R"(\s*\(\s*\".*?\"\s*\)\s*;?\s*(?://.*)?$)");

        int k = sig_idx - 1;
        int steps = 0;

        while (k >= 0 && steps < 60) {
            std::string t = trim(lines[k]);

            if (t.empty()) { --k; ++steps; continue; }             // blank
            if (t.rfind("//", 0) == 0) { --k; ++steps; continue; } // comment

            if (std::regex_match(lines[k], at_line)) {
                if (g_debug) std::cerr << "[debug] removed old " << kAnnotMacro
                                       << "() at line " << k+1 << "\n";
                lines.erase(lines.begin() + k);
                --sig_idx;
                removed = true;
            }
            break;
        }
    }

    // 1) Remove preceding standalone WCET_ANNOTATE(...) lines (if directly tied)
    {
        int k = sig_idx - 1;
        int steps = 0;
        while (k >= 0 && steps < 60) {
            std::string t = trim(lines[k]);

            if (t.empty()) { --k; ++steps; continue; }
            if (t.rfind("//", 0) == 0) { --k; ++steps; continue; }

            if (!t.empty() && t.rfind("WCET_ANNOTATE", 0) == 0) {
                if (g_debug) std::cerr << "[debug] removed old WCET_ANNOTATE at line " << k+1 << "\n";
                lines.erase(lines.begin() + k);
                --sig_idx;
                removed = true;
            }
            break;
        }
    }

    // 2) Remove preceding #pragma wcet next "..." (if directly tied)
    {
        static const std::regex pragma_line(R"(^\s*#\s*pragma\s+wcet\s+next\s*\".*\"\s*$)");
        int k = sig_idx - 1;
        int steps = 0;
        while (k >= 0 && steps < 60) {
            std::string t = trim(lines[k]);

            if (t.empty()) { --k; ++steps; continue; }
            if (t.rfind("//", 0) == 0) { --k; ++steps; continue; }

            if (std::regex_match(lines[k], pragma_line)) {
                if (g_debug) std::cerr << "[debug] removed old #pragma wcet next at line " << k+1 << "\n";
                lines.erase(lines.begin() + k);
                --sig_idx;
                removed = true;
            }
            break;
        }
    }

    // 3) Strip any inline [[gnu::annotate("...")]] on the *signature line itself* (rare here)
    static const std::regex annotate_inline(R"(\[\[\s*gnu::annotate\s*\(\s*\".*?\"\s*\)\s*\]\]\s*)");
    if (sig_idx >= 0 && sig_idx < (int)lines.size()) {
        std::string before = lines[sig_idx];
        std::string after  = std::regex_replace(before, annotate_inline, std::string());
        if (after != before) {
            if (g_debug) std::cerr << "[debug] stripped inline [[gnu::annotate]] at line " << sig_idx+1 << "\n";
            lines[sig_idx] = after;
            removed = true;
        }
    }

    return removed;
}

// Only keep: wcet_ms_exact and cycles
// EDIT: floor wcet_ms_exact to 6 decimal places before emitting.
static std::string make_annotation_line(const Rec& r){
    const double scale = 1e6;
    const double wcet_floor = std::floor(r.wcet_ms_exact * scale) / scale;

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(6);

    oss << kAnnotMacro << "(\"wcet_ms_exact=" << wcet_floor
        << "; cycles=" << r.cycles
        << "\")";

    if (kAnnotNeedsSemicolon) oss << ";";
    return oss.str(); // NO trailing '\n'
}

static std::string make_missing_annotation_line(bool na_as_minus1){
    std::ostringstream oss;
    if (na_as_minus1) {
        oss << kAnnotMacro << "(\"wcet_ms_exact=NaA; cycles=NaA\")";
    } else {
        oss << kAnnotMacro << "(\"wcet_ms_exact=N/A; cycles=N/A\")";
    }
    if (kAnnotNeedsSemicolon) oss << ";";
    return oss.str();
}

static bool is_control_statement_line(const std::string& t_trim){
    // Heuristic: skip common control statements that contain '('
    static const char* kw[] = {"if", "for", "while", "switch", "catch"};
    for (auto* k : kw) {
        std::string k1 = std::string(k) + " ";
        std::string k2 = std::string(k) + "(";
        if (t_trim.rfind(k1, 0) == 0) return true;
        if (t_trim.rfind(k2, 0) == 0) return true;
    }
    return false;
}

static bool has_wcet_at_directly_above(const std::vector<std::string>& lines, int insert_at){
    std::regex at_line(std::string(R"(^\s*)") + kAnnotMacro +
                       R"(\s*\(\s*\".*?\"\s*\)\s*;?\s*(?://.*)?$)");
    int k = insert_at - 1;
    int steps = 0;
    while (k >= 0 && steps < 60) {
        std::string t = trim(lines[k]);
        if (t.empty()) { --k; ++steps; continue; }
        if (t.rfind("//", 0) == 0) { --k; ++steps; continue; }
        return std::regex_match(lines[k], at_line);
    }
    return false;
}

// A light heuristic to avoid annotating function *calls* that look like signatures.
// We require that the candidate region eventually has '{' before ';' (find_definition_brace),
// and that the line doesn't end with ')' only (common in calls), but this is still heuristic.
static bool looks_like_signature_candidate(const std::string& t_trim){
    if (t_trim.empty()) return false;
    if (t_trim[0] == '#') return false;               // preprocessor
    if (t_trim.rfind("return ", 0) == 0) return false; // return foo(...)
    if (t_trim.find('(') == std::string::npos) return false;
    if (is_control_statement_line(t_trim)) return false;

    // Common call-like patterns to skip
    if (t_trim.rfind("LOG", 0) == 0) return false;
    if (t_trim.rfind("printf", 0) == 0) return false;

    return true;
}

static bool is_class_or_struct_header(const std::string& t) {
    // Simple heuristic: "class X" or "struct X"
    // Works for most normal codebases.
    return (t.rfind("class ", 0) == 0) || (t.rfind("struct ", 0) == 0);
}

static bool contains_double_colon(const std::string& s) {
    return s.find("::") != std::string::npos;
}

int main(int argc, char** argv){
    bool backup=false;
    bool annotate_missing=false;
    bool na_as_minus1=false;

    if (argc<3){
        std::cerr<<"Usage: "<<argv[0]<<" [--backup] [--debug] [--annotate-missing] [--na-as-minus1] wcet.json input.cpp\n";
        return 1;
    }

    int ai=1;
    while (ai < argc && argv[ai][0]=='-') {
        std::string flag = argv[ai];
        if (flag=="--backup") backup=true;
        else if (flag=="--debug") g_debug=true;
        else if (flag=="--annotate-missing") annotate_missing=true;
        else if (flag=="--na-as-minus1") na_as_minus1=true;
        else break;
        ++ai;
    }

    if (argc-ai!=2){
        std::cerr<<"Usage: "<<argv[0]<<" [--backup] [--debug] [--annotate-missing] [--na-as-minus1] wcet.json input.cpp\n";
        return 1;
    }

    fs::path jpath = argv[ai];
    fs::path spath = argv[ai+1];

    // ---------- load json ----------
    json root;
    {
        std::ifstream jf(jpath);
        if(!jf){ std::cerr<<"Cannot open "<<jpath<<"\n"; return 1; }
        jf>>root;
    }

    // EDIT: keep ONLY the record with the LOWEST wcet_ms_exact for each function key
    std::unordered_map<std::string, Rec> best_by_func;

    if (root.contains("records") && root["records"].is_array()){
        for(auto& e: root["records"]){
            Rec r;
            if (e.contains("demangled")) r.demangled = e["demangled"].get<std::string>();
            if (e.contains("mangled"))   r.mangled   = e["mangled"].get<std::string>();
            if (e.contains("cycles"))    r.cycles    = e["cycles"].get<long long>();
            if (e.contains("wcet_us_ceil")) r.wcet_us_ceil = e["wcet_us_ceil"].get<long long>();
            if (e.contains("wcet_ms_exact")) r.wcet_ms_exact = e["wcet_ms_exact"].get<double>();

            if (r.demangled.empty())
                continue;

            const std::string key = func_key_from_demangled(r.demangled);
            auto it = best_by_func.find(key);
            if (it == best_by_func.end()) {
                best_by_func.emplace(key, std::move(r));
            } else {
                if (r.wcet_ms_exact < it->second.wcet_ms_exact) {
                    it->second = std::move(r);
                }
            }
        }
    }

    std::vector<Rec> recs;
    recs.reserve(best_by_func.size());
    for (auto& kv : best_by_func) recs.push_back(std::move(kv.second));

    // ---------- read source (store without '\n') ----------
    std::vector<std::string> lines;
    {
        std::ifstream sf(spath);
        if(!sf){ std::cerr<<"Cannot open "<<spath<<"\n"; return 1; }
        std::string s;
        while(std::getline(sf,s)) lines.push_back(s); // NO "\n"
    }

    // ---------- helper: rebuild whole file (with '\n' between lines) ----------
    auto rebuild_whole = [&](std::string& whole){
        whole.clear();
        whole.reserve(lines.size() * 80);
        for (size_t i=0;i<lines.size();++i){
            whole += lines[i];
            whole += '\n';
        }
    };

    std::string whole;
    rebuild_whole(whole);

    auto line_from_pos = [&](size_t pos){
        int line=0;
        for(size_t i=0;i<pos && i<whole.size();++i)
            if(whole[i]=='\n') ++line;
        return line;
    };

    // Track "signature start" lines that got real WCET inserted/replaced
    std::unordered_set<int> inserted_at_lines;

    int matched=0, inserted=0, replaced=0, decl_skips=0;

    // ---------- build regex patterns ----------
    struct Pat { std::regex q; Rec rec; };
    std::vector<Pat> pats;
    pats.reserve(recs.size());
    for (auto& r : recs)
        pats.push_back({ build_qualified_regex(r.demangled), r });

    // ---------- process EACH WCET record ----------
    for (const auto& p : pats){
        std::smatch m;
        if (!std::regex_search(whole, m, p.q)){
            if (g_debug)
                std::cerr<<"[debug] NOT FOUND: "<<p.rec.demangled<<"\n";
            continue;
        }

        ++matched;

        int sig_line = line_from_pos((size_t)m.position());
        if (sig_line < 0 || sig_line >= (int)lines.size()) {
            if (g_debug) std::cerr << "[debug] bad sig_line computed for: " << p.rec.demangled << "\n";
            continue;
        }

        // Confirm it is a DEFINITION by finding '{' (and skipping declarations).
        int brace_line = find_definition_brace(lines, sig_line);
        if (brace_line < 0){
            ++decl_skips;
            if (g_debug)
                std::cerr<<"[debug] declaration skip: "<<p.rec.demangled<<"\n";
            continue;
        }

        // Insert ABOVE the function signature block
        int insert_at = rewind_to_signature_start(lines, sig_line);

        // Remove existing annotation tied to this signature (above it)
        bool was = remove_attr_near(lines, insert_at);

        std::string annot = make_annotation_line(p.rec);

        if (g_debug)
            std::cerr<<"[debug] inserting before signature line "<<insert_at+1
                     <<" (matched line "<<sig_line+1<<", brace line "<<brace_line+1<<")"
                     <<" -> "<<p.rec.demangled<<"\n";

        lines.insert(lines.begin()+insert_at, annot);
        inserted_at_lines.insert(insert_at);

        ++inserted;
        if (was) ++replaced;

        rebuild_whole(whole);
    }

    int missing_inserted = 0;

    // ---------- OPTIONAL: annotate missing definitions ----------
    if (annotate_missing) {
        std::string na_line = make_missing_annotation_line(na_as_minus1);

        bool in_class_like = false;   // inside class/struct body
        int  brace_depth   = 0;       // overall brace depth (rough)

        for (int i = 0; i < (int)lines.size(); ++i) {
            std::string t = trim(lines[i]);
            if (t.empty()) continue;

            // Detect entering a class/struct block (heuristic):
            // if we see "class X" or "struct X", we arm a flag until we hit '{'
            static bool pending_class = false;
            if (!in_class_like && is_class_or_struct_header(t)) {
                pending_class = true;
            }

            // Update brace depth; also use it to toggle in_class_like
            // (We do this before checking signatures so state is correct.)
            for (char ch : lines[i]) {
                if (ch == '{') {
                    ++brace_depth;
                    if (pending_class) {
                        in_class_like = true;
                        pending_class = false;
                    }
                } else if (ch == '}') {
                    // leaving a scope
                    if (brace_depth > 0) --brace_depth;

                    // If we were in a class-like scope and braces drop,
                    // we *may* have exited the class. Heuristic:
                    // when we see a '};' line, end class scope.
                    if (in_class_like && t.find("};") != std::string::npos) {
                        in_class_like = false;
                    }
                }
            }

            // Only add "missing" annotations OUTSIDE class bodies
            if (in_class_like) continue;

            // Candidate must look like a signature
            if (!looks_like_signature_candidate(t)) continue;

            // Strong filter: only out-of-class methods (has ::)
            // This prevents annotating lots of free functions if you don't want them.
            if (!contains_double_colon(t)) continue;

            // Must be a DEFINITION (brace before semicolon)
            int brace_line = find_definition_brace(lines, i);
            if (brace_line < 0) continue;

            int insert_at = rewind_to_signature_start(lines, i);

            // If already has WCET_AT directly above, skip
            if (has_wcet_at_directly_above(lines, insert_at)) continue;

            if (g_debug) {
                std::cerr << "[debug] missing(out-of-class): inserting N/A before line "
                          << insert_at+1 << " (sig " << i+1 << ", brace " << brace_line+1 << ")\n";
            }

            lines.insert(lines.begin() + insert_at, na_line);
            ++missing_inserted;

            // We inserted a line; adjust i so we don't immediately reprocess the same function
            ++i;
        }
    }

    if (g_debug){
        std::cerr<<"[debug] matched="<<matched
                 <<" inserted="<<inserted
                 <<" replaced="<<replaced
                 <<" decl_skips="<<decl_skips
                 <<" missing_inserted="<<missing_inserted
                 <<"\n";
    }

    // ---------- write file back ----------
    fs::path tmp = spath;
    tmp += ".tmp___wcet";

    {
        std::ofstream out(tmp, std::ios::trunc);
        for (auto& L: lines) out << L << "\n";
    }

    if (backup){
        std::error_code ec;
        fs::copy_file(spath, spath.string()+".bak",
                      fs::copy_options::overwrite_existing, ec);
        if (ec) std::cerr<<"Warning: backup failed\n";
    }

    std::error_code ec;
    fs::remove(spath, ec);
    fs::rename(tmp, spath, ec);

    if (ec){
        std::ifstream in(tmp, std::ios::binary);
        std::ofstream out(spath, std::ios::binary|std::ios::trunc);
        out<<in.rdbuf();
        fs::remove(tmp);
    }

    std::cerr<<"[annotate_wcet] inserted "<<inserted
             <<" annotation(s).";
    if (annotate_missing) {
        std::cerr<<" + "<<missing_inserted<<" missing(N/A) annotation(s).";
    }
    std::cerr<<"\n";

    return 0;
}