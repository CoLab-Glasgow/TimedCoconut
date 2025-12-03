#include "gcc-plugin.h"
#include "plugin-version.h"
#include "tree.h"
#include "tree-cfg.h"  
#include "tree-cfg.h"      // label_to_block_fn, CASE_* accessors

#include "gimple.h"
#include "cp/cp-tree.h"
#include "tree-pass.h"
#include "context.h"
#include "tree-iterator.h"
#include "diagnostic.h"
#include "gimple-iterator.h"
#include "cgraph.h"
#include "function.h"
#include <cxxabi.h>
#include <iostream>
#include <cstring>
#include "cfgloop.h"
#include <assert.h>
#include "cfghooks.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <tuple>
#include <string>
#include <queue>
#include <map>
#include "tree-dump.h"
#include <set>
#include "tree-pretty-print.h"
#include <fstream>
#include "basic-block.h"
#include "print-tree.h"
#include "dumpfile.h"
#include <set>
#include <sstream>

#include <cmath>      // for std::fabs
#include <bitset>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>
#include "wide-int.h"   // for wide_int / wi::

#include "tree.h"
#include "cgraph.h"
#include "symtab.h"

#include <string>
#include <vector>

#include <cstdio>
#include <cstring>
#include <cstdlib>

 long depth_r =0;
// Put near your includes, once.
#include "tree.h"

// Portable wrapper: works on GCC 7 (needs precision) and newer (no precision).
#ifndef TO_WIDE
#  if defined(GCCPLUGIN_VERSION) && (GCCPLUGIN_VERSION < 8000)
// GCC 7.x plugin headers
#    define TO_WIDE(T_) wi::to_wide((T_), TYPE_PRECISION (TREE_TYPE (T_)))
#  else
// GCC 8+ (or anything that provides the 1-arg overload)
#    define TO_WIDE(T_) wi::to_wide((T_))
#  endif
#endif





int plugin_is_GPL_compatible = 1;

struct AnalysisContextKey {
    tree fn_decl;
    tree obj;
    int state;
    const char* alias_head;
    bool operator==(AnalysisContextKey const& o) const {
        return fn_decl==o.fn_decl
            && obj    ==o.obj
            && state  ==o.state
            && strcmp(alias_head,o.alias_head)==0;
    }
};
namespace std {
    template<> struct hash<AnalysisContextKey> {
        size_t operator()(AnalysisContextKey const& k) const noexcept {
            return (size_t)k.fn_decl
                 ^ (size_t)k.obj
                 ^ hash<int>()(k.state)
                 ^ hash<const char*>()(k.alias_head);
        }
    };
}

using ParamMap  = std::unordered_map<tree,tree>;
using WorkItem  = std::tuple<tree, ParamMap, AnalysisContextKey>;

static std::unordered_set<AnalysisContextKey> analyzed_contexts;
static std::queue<WorkItem>               work_queue;
static std::map<int, tree> EnumToClassMap;

std::unordered_set<std::string> g_recursive_method_transitions;
static std::map<int, std::vector<std::pair<std::string, int>>> Typestate_Rules;
static std::vector<std::string> TypestateClassConnector_args;
std::unordered_map<tree, std::map<std::string, int>> FSM_by_Class;

std::queue<std::pair<tree, std::unordered_map<tree, tree>>> functions_to_Analyse_with_args;
std::unordered_set<tree> functions_Analysed;
struct CallContext {
     std::unordered_map<tree, tree> alias_map;  // Local alias map for the function
};
std::vector<CallContext> call_stack;  
std::unordered_set<tree> function_params;
std::unordered_set<tree> class_member_vars;
//std::unordered_map<tree, tree> alias_map ;
std::unordered_map<tree, tree> global_alias_map;
std::unordered_map<tree, std::unordered_set<tree>> alias_map;


static std::unordered_set<tree> g_recursive_fn_decls;

// Which transitions are recursive (state self-loops)
struct RecursiveTransitionInfo {
    std::string method;   // function name, e.g. "AdjustActuators"
    int state;            // state id where it self-loops
};

// Filled by validate_wcet_vs_timed_rules when it sees self-loops
extern std::vector<RecursiveTransitionInfo> g_recursive_transitions;

// Fixed recursion bounds per function (filled from main or config)
extern std::unordered_map<std::string, int> g_recursion_bound_by_method;


std::vector<RecursiveTransitionInfo> g_recursive_transitions;
std::unordered_map<std::string, int> g_recursion_bound_by_method;

static std::unordered_map<gimple*, int> condition_id_map;
static int next_if_else_id = 0;
std::map<int, bool> if_else_status;
std::map<int, std::pair<int,int>> if_else_context_map;
static std::vector<std::tuple<tree, gimple*, tree>> deferred_branch_statements;
static std::unordered_map<tree, int> branch_context_map;  
static bool is_new_branch = true;                        
static int current_branch_id = -1;  
static int branch_counter = 0;    
int current_if_else_id = 0;  
std::set<int> joint_states;
std::map<int, bool> branch_context_status;    
static int last_finalized_if_else_id = -1;
static int last_finalized_first_state = -1;

static int g_cycle_n = -1;   // Cycle<N> -> N (e.g. 20)
// --- Helpers to read class-template name and its template args
static const char* class_template_name(tree ty) {
    if (!ty || TREE_CODE(ty) != RECORD_TYPE) return nullptr;
    tree td = TYPE_NAME(ty);
    if (!td || TREE_CODE(td) != TYPE_DECL) return nullptr;
    tree id = DECL_NAME(td);
    return id ? IDENTIFIER_POINTER(id) : nullptr;
}

static tree class_template_args(tree ty) {
    if (!ty || TREE_CODE(ty) != RECORD_TYPE) return nullptr;
    if (!TYPE_TEMPLATE_INFO(ty)) return nullptr;
    return TI_ARGS(TYPE_TEMPLATE_INFO(ty)); // TREE_VEC
}


// Per-edge timing info
struct TimedInfo {
    long long period_ns  = 0;   // period
    long long deadline_ns= 0;   // relative deadline
    int criticality      = 0;   // enum integral value
    long long jitter_ns  = 0;   // optional last param
};

// from_state -> [(method, next_state, timing)]
static std::map<int,
    std::vector<std::tuple<std::string,int,TimedInfo>>
> Timed_Typestate_Rules;




static bool as_sll(tree t, long long &out) {
    if (!t || TREE_CODE(t) != INTEGER_CST) return false;
    out = (long long)TREE_INT_CST_LOW(t);
    return true;
}

static tree get_template_args_from_type(tree ty) {
    if (!ty) return nullptr;
    if (TREE_CODE(ty) == RECORD_TYPE && TYPE_TEMPLATE_INFO(ty))
        return TI_ARGS(TYPE_TEMPLATE_INFO(ty));
    return nullptr;
}

// TimeGuard<Release, Period, Deadline, Criticality, Jitter?>
static bool parse_timeguard(tree tg, TimedInfo &ti) {
    tree args = nullptr;

    // TimeGuard is a template-id type; get its args
    if (TYPE_P(tg)) args = get_template_args_from_type(tg);
    if (!args || TREE_CODE(args) != TREE_VEC) return false;

    const int n = TREE_VEC_LENGTH(args);
    if (n < 4) return false;

    long long  p=0, d=0, c=0, j=0;
    tree a0 = TREE_VEC_ELT(args,0);
    tree a1 = TREE_VEC_ELT(args,1);
    tree a2 = TREE_VEC_ELT(args,2);
    tree a3 = TREE_VEC_ELT(args,3);
    tree a4 = (n >= 5 ? TREE_VEC_ELT(args,4) : nullptr);

    as_sll(a0, p);
    as_sll(a1, d);
   // as_sll(a2, c);
    if (a4) as_sll(a4, j);

   
    if (TREE_CODE(a2) == INTEGER_CST) {
        c = (int)TREE_INT_CST_LOW(a2);
    } else if (TREE_CODE(a2) == CONST_DECL) {
        tree v = DECL_INITIAL(a2);
        if (v && TREE_CODE(v) == INTEGER_CST) c = (int)TREE_INT_CST_LOW(v);
    }

  
    ti.period_ns   = p;
    ti.deadline_ns = d;
    ti.criticality = c;
    ti.jitter_ns   = j;
    return true;
}


// Timed_State<STATE, &Class::method, C<>, TimeGuard<...>, NEXT>

static void process_timed_typestate_args(tree targs) {
    if (!targs || TREE_CODE(targs) != TREE_VEC || TREE_VEC_LENGTH(targs) < 4) return;

    int from = -1, to = -1 ;
    std::string method_name;
    TimedInfo tinfo{};
    tree class_tree = nullptr;

    // 0) from-state enum value
    tree a0 = TREE_VEC_ELT(targs, 0);
    if (a0 && TREE_CODE(a0) == INTEGER_CST)
        from = (int)TREE_INT_CST_LOW(a0);

    // 1) &Class::method
    tree a1 = TREE_VEC_ELT(targs, 1);
    if (a1 && TREE_CODE(a1) == PTRMEM_CST) {
        tree member = PTRMEM_CST_MEMBER(a1);
        if (member && TREE_CODE(member) == FUNCTION_DECL) {
            class_tree = DECL_CONTEXT(member);
            if (DECL_NAME(member)) {
                const char* nm = IDENTIFIER_POINTER(DECL_NAME(member));
                if (nm) method_name = nm;
            }
        }
    }
   
    // 2) TimeGuard<...>
    tree a2 = TREE_VEC_ELT(targs, 2);
    parse_timeguard(a2, tinfo); // best-effort; 0s mean "unspecified"

    // 3) to-state enum value
    tree a3 = TREE_VEC_ELT(targs, 3);
    if (a3 && TREE_CODE(a3) == INTEGER_CST)
        to = (int)TREE_INT_CST_LOW(a3);

    if (from != -1 && !method_name.empty() && to != -1) {
        Timed_Typestate_Rules[from].emplace_back(method_name, to, tinfo);

        // (optional) keep your per-class FSM map in sync
        if (class_tree && TREE_CODE(class_tree) == RECORD_TYPE) {
            FSM_by_Class[class_tree][method_name] = to;
        }

        // (optional) map enum value → class 
        EnumToClassMap[from] = class_tree;
    }
}









static void print_timed_typestate_rules() {

    printf("Timed Typestate Rules\n");
    for (const auto &[from, edges] : Timed_Typestate_Rules) {
        printf("From state %d:\n", from);

        for (const auto &edge : edges) {
            const std::string &method = std::get<0>(edge);
            int to = std::get<1>(edge);
            const TimedInfo &ti = std::get<2>(edge);

            printf("  --[%s]--> %d\n", method.c_str(), to);
            printf("     Timing: Period=%lld, Deadline=%lld, offest=%lld, Criticality=%d\n",
                   ti.period_ns, ti.deadline_ns, ti.jitter_ns, ti.criticality);
        }
    }

    
}






















bool is_subclass(tree subclass_tree, tree superclass_tree) {
    if (subclass_tree == superclass_tree) {
        return false;  // A class is not a subclass of itself
    }

    tree subclass_binfo = TYPE_BINFO(subclass_tree);
    if (!subclass_binfo) {
        return false;  // No base information, cannot be a subclass
    }

    // Iterate over all base classes using BINFO_BASE_ITERATE
    for (int i = 0; i < BINFO_N_BASE_BINFOS(subclass_binfo); i++) {
        tree base = BINFO_BASE_BINFO(subclass_binfo, i);
        tree base_type = BINFO_TYPE(base);
        if (base_type == superclass_tree) {
            return true;  // Found superclass in the inheritance chain
        }
    }

    return false;
}





/**
 * This following functions are responsible for extracting typestate rules from Typestate_Templates and saving them for validation.
 * They extract the arguments of TypestateClassConnector to tag the type that is subject to typestate validation.
 * Lastly, it visualises the typestate rules in the form of an FSM using Graphviz.
 */

static std::map<int, std::vector<std::pair<std::string, int>>> Subtype_Typestate_Rules;


void process_typestate_template_args(tree tmpl_args) {
    int num_args = TREE_VEC_LENGTH(tmpl_args);
    int current_enum_value = -1;
    std::string enum_type_name;
    tree arg_type = nullptr;

    // First, extract the current enum value and its type name
    if (num_args >= 1) {
        tree arg = TREE_VEC_ELT(tmpl_args, 0);
        if (TREE_CODE(arg) == INTEGER_CST) {
            arg_type = TREE_TYPE(arg);
            if (arg_type && TREE_CODE(arg_type) == ENUMERAL_TYPE) {
                HOST_WIDE_INT arg_val = TREE_INT_CST_LOW(arg);
                current_enum_value = static_cast<int>(arg_val);

                if (TYPE_NAME(arg_type)) {
                    const char* type_name_cstr = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(arg_type)));
                    enum_type_name = type_name_cstr ? type_name_cstr : "<unknown_enum_type>";
                } else {
                    enum_type_name = "<unknown_enum_type>";
                }
            }
        }
    }

    if (current_enum_value == -1 || enum_type_name.empty()) {
        return;  // Exit if enum extraction failed
    }

    std::vector<std::pair<std::string, int>> transitions;
    tree class_tree = nullptr;  // To store the class type of the current function

    for (int i = 1; i + 1 < num_args; i += 2) {
        std::string function_name;
        int next_enum_value = -1;
        tree func_arg = TREE_VEC_ELT(tmpl_args, i);

        // Extract the class name from the function pointer
        if (TREE_CODE(func_arg) == PTRMEM_CST) {
            tree member = PTRMEM_CST_MEMBER(func_arg);
            if (TREE_CODE(member) == FUNCTION_DECL) {
                class_tree = DECL_CONTEXT(member);  // Get the class type
                const char* func_name = (DECL_NAME(member)) ? IDENTIFIER_POINTER(DECL_NAME(member)) : "<unnamed_function>";
                function_name = func_name;
            }
        }

        // Extract next enum value
        tree next_state_arg = TREE_VEC_ELT(tmpl_args, i + 1);
        if (TREE_CODE(next_state_arg) == INTEGER_CST) {
            tree next_arg_type = TREE_TYPE(next_state_arg);
            if (next_arg_type && TREE_CODE(next_arg_type) == ENUMERAL_TYPE) {
                HOST_WIDE_INT arg_val = TREE_INT_CST_LOW(next_state_arg);
                next_enum_value = static_cast<int>(arg_val);
            }
        }

        if (!function_name.empty() && next_enum_value != -1) {
            transitions.emplace_back(function_name, next_enum_value);
        }
    }

    // Store the class-tree mapping for the current enum value
    EnumToClassMap[current_enum_value] = class_tree;

    // Append transitions to the appropriate map
    Typestate_Rules[current_enum_value].insert(
        Typestate_Rules[current_enum_value].end(),
        transitions.begin(),
        transitions.end());

    //  Also track FSM by class tree for LSP checking
    if (class_tree && TREE_CODE(class_tree) == RECORD_TYPE) {
        for (const auto& [method_name, next_state] : transitions) {
            if (!method_name.empty()) {
                FSM_by_Class[class_tree][method_name] = next_state;
            }
        }
    }
}


static inline tree canon_type(tree t) {
    return t ? TYPE_MAIN_VARIANT(t) : t;
}


// ADD: pragma support + small containers
#include "c-family/c-pragma.h"
#include <deque>
#include <string>
static std::deque<std::string> g_wcet_pending;

static inline bool is_fn_definition(tree fndecl){
    return fndecl && TREE_CODE(fndecl)==FUNCTION_DECL
           && DECL_INITIAL(fndecl) && !DECL_EXTERNAL(fndecl);
}
static inline bool is_from_system_header(tree t){
    return in_system_header_at(DECL_SOURCE_LOCATION(t));
}
static inline bool is_artificial(tree fndecl){
    return DECL_ARTIFICIAL(fndecl);
}

static std::string qualified_name_for_decl(tree fndecl){
    if (!fndecl || TREE_CODE(fndecl)!=FUNCTION_DECL) return "<fn>";
    const char* base = DECL_NAME(fndecl) ? IDENTIFIER_POINTER(DECL_NAME(fndecl)) : "<fn>";
    std::string name = base ? base : "<fn>";
    tree ctx = DECL_CONTEXT(fndecl);
    if (ctx && TREE_CODE(ctx)==RECORD_TYPE) {
        tree tn = TYPE_NAME(ctx);
        if (tn && TREE_CODE(tn)==TYPE_DECL) {
            tree id = DECL_NAME(tn);
            if (id) name = std::string(IDENTIFIER_POINTER(id)) + "::" + name;
        }
    }
    return name;
}

// Parse fields out of the payload string (simple, tolerant)
struct AnnotRow {
    std::string qualified, file, method;
    int line = 0;
    double wcet_ms = 0.0;
    long long cycles=0, us_ceil=0, f_cpu=0;
};
static bool parse_double_field(const char* s, const char* key, double& out){
    const char* p = strstr(s, key); if (!p) return false;
    p += strlen(key); while (*p==' '||*p=='\t') ++p;
    char* e=nullptr; double v=strtod(p,&e); if (e==p) return false; out=v; return true;
}
static bool parse_ll_field(const char* s, const char* key, long long& out){
    const char* p = strstr(s, key); if (!p) return false;
    p += strlen(key); while (*p==' '||*p=='\t') ++p;
    char* e=nullptr; long long v=strtoll(p,&e,10); if (e==p) return false; out=v; return true;
}
static bool parse_payload(const char* s, AnnotRow& row){
    if (!s) return false;
    bool ok = parse_double_field(s, "wcet_ms_exact=", row.wcet_ms);
    parse_ll_field(s, "cycles=",  row.cycles);
    parse_ll_field(s, "us_ceil=", row.us_ceil);
    parse_ll_field(s, "F_CPU=",   row.f_cpu);
    return ok;
}


static std::vector<AnnotRow> g_rows;
static std::unordered_map<const void*, std::unordered_map<std::string, double>> g_annot_wcet_ms; // class -> method -> ms
static std::vector<AnnotRow> g_annot_rows; // for printing a list

// ADD if not present
#include <cmath>

static inline const void* canon_key_from_record(tree t){
    return (t && TREE_CODE(t) == RECORD_TYPE) ? (const void*) TYPE_MAIN_VARIANT(t) : (const void*) t;
}

static void record_wcet_row(const void* class_key_canon_in,
                            const std::string& method,
                            tree fndecl,
                            double ms)
{
    // Canonicalise (safe even if already canonical)
    const void* class_key_canon = class_key_canon_in;

    // If caller passed a raw RECORD_TYPE, canonicalise here too:
    if (!class_key_canon && fndecl) {
        tree ctx = DECL_CONTEXT(fndecl);
        if (ctx && TREE_CODE(ctx) == RECORD_TYPE)
            class_key_canon = (const void*) TYPE_MAIN_VARIANT(ctx);
    }

    // 1) WRITE TO THE MAP  (this is what the validator reads)
    g_annot_wcet_ms[class_key_canon][method] = ms;

    // 2) Also keep a row for pretty printing
    AnnotRow row;
    row.wcet_ms  = ms;
    row.method   = method;
    row.qualified= qualified_name_for_decl(fndecl);
    if (fndecl) {
        location_t loc = DECL_SOURCE_LOCATION(fndecl);
        row.file = LOCATION_FILE(loc) ? LOCATION_FILE(loc) : "";
        row.line = LOCATION_LINE(loc);
    }
    g_rows.push_back(row);

    // Optional debug
   //  fprintf(stderr, "[wcet] attached %.6f ms to %s\n", ms, row.qualified.c_str());
}


// Parse "wcet_ms_exact=..." out of the payload string
static bool parse_wcet_ms_from_annot(const char* msg, double& out_ms) {
    if (!msg) return false;
    const char* p = strstr(msg, "wcet_ms_exact=");
    if (!p) return false;
    p += strlen("wcet_ms_exact=");
    while (*p == ' ' || *p == '\t') ++p;

    char* endp = nullptr;
    double v = strtod(p, &endp);
    if (endp == p) return false;

    out_ms = v;
    return true;
}


static void dump_attrs_for_fn(const char* tag, tree fndecl) {
    auto dump_list = [](tree attrs) {
        for (tree a = attrs; a; a = TREE_CHAIN(a)) {
            tree purpose = TREE_PURPOSE(a);
            const char* aname = (purpose && TREE_CODE(purpose) == IDENTIFIER_NODE)
                                ? IDENTIFIER_POINTER(purpose) : "<non-id>";
            fprintf(stderr, "    attr: %s\n", aname ? aname : "<null>");

            tree args = TREE_VALUE(a);
            if (!args) continue;

            // single value?
            if (TREE_CODE(args) == STRING_CST) {
                fprintf(stderr, "      arg[string]: \"%s\"\n", TREE_STRING_POINTER(args));
            } else if (TREE_CODE(args) == INTEGER_CST) {
                fprintf(stderr, "      arg[int]: %lld\n", (long long)TREE_INT_CST_LOW(args));
            } else if (TREE_CODE(args) == TREE_LIST) {
                for (tree it = args; it; it = TREE_CHAIN(it)) {
                    tree v = TREE_VALUE(it);
                    if (!v) continue;
                    if (TREE_CODE(v) == STRING_CST) {
                        fprintf(stderr, "      list[string]: \"%s\"\n", TREE_STRING_POINTER(v));
                    } else if (TREE_CODE(v) == INTEGER_CST) {
                        fprintf(stderr, "      list[int]: %lld\n", (long long)TREE_INT_CST_LOW(v));
                    } else {
                        fprintf(stderr, "      list[node code=%d]\n", (int)TREE_CODE(v));
                    }
                }
            } else {
                fprintf(stderr, "      args node code=%d\n", (int)TREE_CODE(args));
            }
        }
    };

    const char* nm = (DECL_NAME(fndecl) ? IDENTIFIER_POINTER(DECL_NAME(fndecl)) : "<noname>");
    fprintf(stderr, "[attrs:%s] %s\n", tag, nm);
    dump_list(DECL_ATTRIBUTES(fndecl));

    tree ft = TREE_TYPE(fndecl);
    if (ft) {
        fprintf(stderr, "[attrs:%s] %s (fn-type)\n", tag, nm);
        dump_list(TYPE_ATTRIBUTES(ft));
        tree rt = TREE_TYPE(ft);
        if (rt) {
            fprintf(stderr, "[attrs:%s] %s (ret-type)\n", tag, nm);
            dump_list(TYPE_ATTRIBUTES(rt));
        }
    }
}

static void wcet_on_finish_decl_wcet(void* data, void* /*user*/) {
       tree decl = static_cast<tree>(data);
    if (!decl) return;

    // 1) If this is our marker: VAR_DECL in section ".wcet_next" with STRING_CST init
    if (TREE_CODE(decl)==VAR_DECL) {
        const char* sec = DECL_SECTION_NAME(decl);
        if (sec && strcmp(sec, ".wcet_next")==0) {
            tree init = DECL_INITIAL(decl);
            if (init && TREE_CODE(init)==STRING_CST) {
                const char* s = TREE_STRING_POINTER(init);
                g_wcet_pending.emplace_back(s);
                fprintf(stderr, "[wcet] marker queued: \"%s\"\n", s);
            }
            return; // nothing else to do for VAR_DECL
        }
    }

    // 2) If this is a function definition, and we have a pending marker, attach it
    if (TREE_CODE(decl)==FUNCTION_DECL && !g_wcet_pending.empty()) {
        if (!is_fn_definition(decl))     return;  // skip prototypes
        if (is_from_system_header(decl)) return;  // skip libstdc++/builtins
        if (is_artificial(decl))         return;  // skip synthesized

        std::string payload = std::move(g_wcet_pending.front());
        g_wcet_pending.pop_front();

        AnnotRow row;
        if (!parse_payload(payload.c_str(), row)) {
            const char* nm = DECL_NAME(decl) ? IDENTIFIER_POINTER(DECL_NAME(decl)) : "<noname>";
            warning(0, "malformed wcet payload for %qs: %qs", nm, payload.c_str());
            return;
        }

        row.qualified = qualified_name_for_decl(decl);
        location_t loc = DECL_SOURCE_LOCATION(decl);
        row.file = LOCATION_FILE(loc) ? LOCATION_FILE(loc) : "";
        row.line = LOCATION_LINE(loc);
        row.method = DECL_NAME(decl) ? IDENTIFIER_POINTER(DECL_NAME(decl)) : "";

        // store/print (you probably already have a vector; reuse it)
 //   std::vector<AnnotRow> g_rows; // or keep it file-static if you prefer
        g_rows.push_back(row);

        fprintf(stderr, "[wcet] attached %.6f ms to %s @ %s:%d\n",
                row.wcet_ms, row.qualified.c_str(), row.file.c_str(), row.line);
    }
}



static void collect_annot_payload_strings(tree args, std::vector<std::string>& out) {
    if (!args) return;

    // Single literal e.g. __attribute__((annotate("...")))
    if (TREE_CODE(args) == STRING_CST) {
        out.emplace_back(TREE_STRING_POINTER(args));
        return;
    }

    // List of arguments
    if (TREE_CODE(args) == TREE_LIST) {
        for (tree it = args; it; it = TREE_CHAIN(it)) {
            tree v = TREE_VALUE(it);
            if (!v) continue;
            if (TREE_CODE(v) == STRING_CST) {
                out.emplace_back(TREE_STRING_POINTER(v));
            }
            // ignore non-string nodes
        }
    }
    // else: ignore other forms
}


static bool harvest_one_attr_list(tree attrs, const void* class_key_raw,
                                  const std::string& method, tree fndecl)
{
    if (!attrs) return false;

    tree class_key = (tree)class_key_raw;
    if (class_key) class_key = TYPE_MAIN_VARIANT(class_key);
    const void* class_key_canon = (const void*)class_key;

    bool found = false;
    for (tree a = attrs; a; a = TREE_CHAIN(a)) {
        tree purpose = TREE_PURPOSE(a);
        if (!purpose || TREE_CODE(purpose) != IDENTIFIER_NODE) continue;
        const char* aname = IDENTIFIER_POINTER(purpose);
        if (!aname) continue;
        if (strcmp(aname, "annotate") && strcmp(aname, "gnu::annotate")) continue;

        std::vector<std::string> payloads;
        collect_annot_payload_strings(TREE_VALUE(a), payloads); // << enabled

        for (const auto& s : payloads) {
            if (s.find("wcet_ms_exact=") == std::string::npos) continue;
            double ms = 0.0;
            if (!parse_wcet_ms_from_annot(s.c_str(), ms)) continue;
            record_wcet_row(class_key_canon, method, fndecl, ms);
            found = true;
        }
    }
    return found;
}


static bool harvest_from_class_declaration(tree class_type,
                                           const std::string& method,
                                           const void* class_key,
                                           tree fndecl_for_name_only)
{
    if (!class_type || TREE_CODE(class_type) != RECORD_TYPE) return false;
    // iterate class members (fields + methods)
    for (tree fld = TYPE_FIELDS(class_type); fld; fld = TREE_CHAIN(fld)) {
        if (TREE_CODE(fld) != FUNCTION_DECL) continue;
        if (!DECL_NAME(fld)) continue;

        const char* nm = IDENTIFIER_POINTER(DECL_NAME(fld));
        if (!nm) continue;

        if (method == nm) {
            // Found the member *declaration*; harvest its attributes
            if (harvest_one_attr_list(DECL_ATTRIBUTES(fld), class_key, method, fndecl_for_name_only))
                return true;
        }
    }
    return false;
}

static void harvest_annot_from_decl(tree fndecl) {
    if (!fndecl || TREE_CODE(fndecl) != FUNCTION_DECL || !DECL_NAME(fndecl)) return;

    std::string method = IDENTIFIER_POINTER(DECL_NAME(fndecl));
    tree ctx = DECL_CONTEXT(fndecl);
    tree class_type = (ctx && TREE_CODE(ctx) == RECORD_TYPE) ? TYPE_MAIN_VARIANT(ctx) : nullptr;
    const void* class_key = (const void*)class_type;

    // 1) attributes on the definition
    if (harvest_one_attr_list(DECL_ATTRIBUTES(fndecl), class_key, method, fndecl)) return;

    // 2) attributes on function type
    tree fn_type = TREE_TYPE(fndecl);
    if (fn_type && harvest_one_attr_list(TYPE_ATTRIBUTES(fn_type), class_key, method, fndecl)) return;

    // 3) attributes on return type
    if (fn_type) {
        tree ret_type = TREE_TYPE(fn_type);
        if (ret_type && harvest_one_attr_list(TYPE_ATTRIBUTES(ret_type), class_key, method, fndecl)) return;
    }

    // 🔴 4) attributes on the *member declaration* inside the class
    if (class_type) {
        if (harvest_from_class_declaration(class_type, method, class_key, fndecl)) return;
    }

    // Debug (optional)
   // fprintf(stderr, "[decl] %s: decl_attrs=%s\n",
        //    IDENTIFIER_POINTER(DECL_NAME(fndecl)),
         //   DECL_ATTRIBUTES(fndecl) ? "yes" : "no");
    //log_decl_annotate_values(fndecl);
}





// ----------------------------------------------------------------------
// Main function: process_typestate_template_pack
// template<typename C, typename TFirst, typename... TRest>
// ----------------------------------------------------------------------
static void process_typestate_template_pack(tree ts_args_vec)
{
    if (!ts_args_vec || TREE_CODE(ts_args_vec) != TREE_VEC)
        return;

    const int N = TREE_VEC_LENGTH(ts_args_vec);
    if (N == 0)
        return;

    // ------------------------------------------------------------------
    // 1. First template argument is the cycle type: C = Cycle<...>
    // ------------------------------------------------------------------
    tree cycle_ty = TREE_VEC_ELT(ts_args_vec, 0);

    if (TYPE_P(cycle_ty) && TREE_CODE(cycle_ty) == RECORD_TYPE) {
        // Extract Cycle<N>’s integer template argument, if present
        tree cycle_args = class_template_args(cycle_ty);
        if (cycle_args && TREE_CODE(cycle_args) == TREE_VEC &&
            TREE_VEC_LENGTH(cycle_args) >= 1)
        {
            tree n = TREE_VEC_ELT(cycle_args, 0);
            if (n && TREE_CODE(n) == INTEGER_CST) {
                HOST_WIDE_INT val = TREE_INT_CST_LOW(n);
                g_cycle_n = (int) val;

                // <<< the only print you asked for >>>
               // fprintf(stderr, "Cycle<N> = %d\n", g_cycle_n);
            }
        }
    }

    // ------------------------------------------------------------------
    // 2. Remaining template arguments are TFirst, TRest... (states)
    // ------------------------------------------------------------------
    for (int i = 1; i < N; ++i) {   // start at 1, skip C
        tree arg_ty = TREE_VEC_ELT(ts_args_vec, i);

        // Each element is itself a class template instantiation: State<...> or Timed_State<...>
        if (!TYPE_P(arg_ty) || TREE_CODE(arg_ty) != RECORD_TYPE)
            continue;

        // Get inner template arguments of this element
        tree inner = class_template_args(arg_ty);
        if (!inner || TREE_CODE(inner) != TREE_VEC)
            continue;

        const int M = TREE_VEC_LENGTH(inner);

        // Heuristic by shape:
        //   State<STATE, &C::m, NEXT>                       -> 3 args
        //   Timed_State<STATE, &C::m, TimeGuard<...>, NEXT> -> 4 args
        if (M == 3) {
            process_typestate_template_args(inner);
        } else if (M == 4) {
            // Additional sanity: check arg1 is &C::m (PTRMEM_CST)
            tree a1 = TREE_VEC_ELT(inner, 1);
            if (a1 && TREE_CODE(a1) == PTRMEM_CST) {
                process_timed_typestate_args(inner);
            }
        }
    }
}


static void harvest_all_functions() {
    cgraph_node* n;
    FOR_EACH_FUNCTION (n) {
        if (!n->decl) continue;
        harvest_annot_from_decl(n->decl);
    }
}

static void print_annotated_wcet_list() {
    if (g_rows.empty()) { printf("WCET list (empty)\n"); return; }

    std::sort(g_rows.begin(), g_rows.end(),
              [](const AnnotRow& a, const AnnotRow& b){ return a.qualified < b.qualified; });

    printf("WCET list\n");
    for (const auto& r : g_rows) {
        printf("  %s : wcet_ms_exact=%.6f\n",
               r.qualified.c_str(), r.wcet_ms);
    }
    fflush(stdout);

   
}




static inline const void* canon_key_from_record(tree t); // from step 1




static const bool   kDebugValidate        = true;   // verbose stderr logging
static const bool   kFailBuildOnViolation = false;  // set true to error out
static const double kAbsTolMs             = 1e-3;   // 1 µs
static const double kRelTol               = 1e-6;   // 1 ppm

// Your TimedInfo fields are in MILLISECONDS:
static inline double to_ms_ll(long long v_ms) { return static_cast<double>(v_ms); }




// Build a fallback WCET map from g_rows (class unknown → nullptr bucket)
static std::unordered_map<const void*, std::unordered_map<std::string,double>>
build_wcet_map_fallback_from_rows()
{
    std::unordered_map<const void*, std::unordered_map<std::string,double>> m;
    for (const auto& r : g_rows) {
        std::string meth = r.method;
        if (meth.empty()) {
            auto pos = r.qualified.rfind("::");
            meth = (pos == std::string::npos) ? r.qualified : r.qualified.substr(pos+2);
        }
        m[nullptr][meth] = r.wcet_ms;
    }
    return m;
}


static void dump_wcet_buckets(
    const std::unordered_map<const void*, std::unordered_map<std::string,double>>& wcet_by_class)
{
    if (!kDebugValidate) return;
   // std::fprintf(stderr, "[validate] ---- WCET buckets dump ----\n");
    if (wcet_by_class.empty()) {
   //     std::fprintf(stderr, "[validate] (empty)\n");
    } else {
        for (const auto& cls : wcet_by_class) {
       //     std::fprintf(stderr, "[validate] class_key=%p%s\n",
               //          cls.first, cls.first ? "" : " (nullptr bucket)");
            for (const auto& kv : cls.second) {
              //  std::fprintf(stderr, "           %-24s -> %.6f ms\n",
                 //            kv.first.c_str(), kv.second);
            }
        }
    }
 //   std::fprintf(stderr, "[validate] --------------------------------\n");
}

// Safe reporter: preformat to buffer, then use warning_at/error_at with "%s"
static void report_violation(bool as_error, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (as_error) {
        error_at(UNKNOWN_LOCATION, "%s", buf);
    } else {
        warning_at(UNKNOWN_LOCATION, 0, "%s", buf);
    }
}



// ======================== FULL VALIDATOR ========================
// -----------------------------------------------------------------------------
// Timing validation per the paper's semantics (HARD/FIRM vs SOFT; sporadic; and
// system-level feasibility via a per-activation timeline).
// -----------------------------------------------------------------------------

enum class Contract { HARD, SOFT }; // FIRM folds into HARD

static inline Contract contract_from_criticality(int crit) {
    switch (crit) {
        case 1: // HARD
        case 2: // FIRM -> enforce like HARD
            return Contract::HARD;
        case 3: // SOFT
            return Contract::SOFT;
        default:
            // Be conservative if unknown
            return Contract::HARD;
    }
}

static inline double ns_to_ms(long long ns) { return to_ms_ll(ns); }

// If period present but no deadline, treat as SPORADIC: period = min inter-arrival,
// not a per-transition budget.
static inline bool is_sporadic(const TimedInfo& ti) {
    return (ti.period_ns > 0 && ti.deadline_ns == 0);
}

// Worst-case release instant (release + jitter)
static inline long long worst_release_ns(const TimedInfo& ti) {
    long long j = ti.jitter_ns;
    if (j < 0) j = 0;
    return  j;
}

// ===== window helpers (milliseconds) =========================================
static const double kTimelineAbsTolMs = 1e-3; // 1 µs tolerance

static inline double current_window_start(double t_now,
                                          double period_ms,
                                          double offset_ms)
{
    if (period_ms <= 0.0) return offset_ms; // aperiodic: offset as anchor
    const double k = std::floor((t_now - offset_ms) / period_ms);
    return k * period_ms + offset_ms;
}

static inline const char* crit_label_int(int crit) {
    switch (crit) {
        case 0: return "SOFT";
        case 1: return "HARD";
        case 2: return "FIRM";
        case 3: return "SOFT";
        default: return "?";
    }
}

// Validate one call inside its period/offset window and advance absolute time.
// Returns true if deadline met within the window; false if missed.
static bool check_edge_timing_ms(int state_id,
                                 const std::string& method,
                                 const TimedInfo& ti,
                                 double wcet_ms,
                                 double& t_abs /* in/out */)
{
    // TimedInfo fields are stored in *milliseconds* in your project:
    const double period_ms   = static_cast<double>(ti.period_ns);
   
    const double deadline_ms = static_cast<double>(ti.deadline_ns);
    const double offset_ms   = static_cast<double>(ti.jitter_ns); // <-- if spelled offest_ns, change here
    const int    crit        = static_cast<int>(ti.criticality);

    const double tol = kTimelineAbsTolMs;

    // Window that contains the *current* absolute time
    const double win_start  = current_window_start(t_abs, period_ms, offset_ms);
      printf("Cycle=%d\n",g_cycle_n);
    // This job's release/deadline inside that window
   // const double job_release  = win_start + release_ms;
   // const double job_deadline = (deadline_ms > 0.0)
                            //  ? (job_release + deadline_ms)
                           //   : (period_ms > 0.0 ? (win_start + period_ms)
                                       //          : std::numeric_limits<double>::infinity());

    // Wait until release if we arrived early
    //if (t_abs + tol < job_release) {
      //  std::fprintf(stderr,
         //   "[timeline] state=%d  wait: t=%.4f -> %.4f ms (win_start=%.4f, rel=%.4f, off=%.4f, per=%.4f)\n",
          //  state_id, t_abs, job_release, win_start, release_ms, offset_ms, period_ms);
     //   t_abs = job_release;
   // }
   float c= g_cycle_n;
    const double t_start = t_abs;
    const double t_end   = t_start + wcet_ms;
    // Progress line (matches your style)
    std::fprintf(stderr,
        "  t=%.4f ms  --%s()-->  t=%.4f ms   (d<%.4f ms)   (Cycle=%.4f ms)  (%s)\n",
        t_start, method.c_str(), t_end,
        (deadline_ms > 0.0 ? deadline_ms : (period_ms > 0.0 ? period_ms : 0.0)),
        c, crit_label_int(crit));

    // Check deadline *within this window*.
   // const bool miss = (t_end > job_deadline + tol);
    if (wcet_ms>g_cycle_n) {
        // Report relative to window to avoid confusion
     //   const double elapsed_end = t_end - win_start;
      error_at(UNKNOWN_LOCATION,
                 "System-level infeasible within cycle exceeds bound %.6f ms "
                 "",
               g_cycle_n);
     //  return false;
     }

   // t_abs = t_end; // advance absolute time
    return true;
}

// Run a windowed timeline for one state's edges (period-window view)
static void run_timeline_for_state_ms(
    int state_id,
    const std::vector<std::tuple<std::string,int,TimedInfo>>& edges,
    const std::unordered_map<std::string,double>& wcet_bucket)
{
    std::fprintf(stderr, "[timeline] state=%d  window=period\n", state_id);

    double t_abs = 0.0; // start-of-frame; windowing uses offset internally
    bool all_ok = true;

    for (const auto& e : edges) {
        const std::string& method = std::get<0>(e);
        const TimedInfo&   ti     = std::get<2>(e);

        auto itW = wcet_bucket.find(method);
        const double wcet_ms = (itW != wcet_bucket.end()) ? itW->second : 0.0;

        const bool ok = check_edge_timing_ms(state_id, method, ti, wcet_ms, t_abs);
        all_ok = all_ok && ok;
    }

    if (all_ok && !edges.empty()) {
        const TimedInfo& last = std::get<2>(edges.back());
        const double period_ms = static_cast<double>(last.period_ns);
       const double offset_ms   = static_cast<double>(last.jitter_ns); // <-- if spelled offest_ns, change here
        if (period_ms > 0.0) {
            const double win_start = current_window_start(t_abs, period_ms, offset_ms);
            const double slack = (win_start + period_ms) - t_abs;
            std::fprintf(stderr,
                "  [timeline] completes at t=%.4f ms within budget %.4f ms (slack=%.4f ms)\n",
                t_abs, period_ms, slack);
        } else {
            std::fprintf(stderr,
                "  [timeline] completes at t=%.4f ms (no period)\n", t_abs);
        }
    }
}

// Emit an ASAP (no waiting) end-to-end trace that follows the first edge each step.
#include <set>

// Deduplicate rules (call this once before validating/printing)
static inline bool same_ti(const TimedInfo& a, const TimedInfo& b) {
    return
           a.period_ns   == b.period_ns   &&
           a.deadline_ns == b.deadline_ns &&
           a.criticality == b.criticality;
}
static void normalize_rules() {
    for (auto& kv : Timed_Typestate_Rules) {
        auto& v = kv.second;
        std::sort(v.begin(), v.end(),
            [](const auto& x, const auto& y){
                if (std::get<0>(x) != std::get<0>(y)) return std::get<0>(x) < std::get<0>(y);
                if (std::get<1>(x) != std::get<1>(y)) return std::get<1>(x) < std::get<1>(y);
                const auto& ax = std::get<2>(x);
                const auto& ay = std::get<2>(y);
              //  if (ax.release_ns  != ay.release_ns)  return ax.release_ns  < ay.release_ns;
                if (ax.period_ns   != ay.period_ns)   return ax.period_ns   < ay.period_ns;
                if (ax.deadline_ns != ay.deadline_ns) return ax.deadline_ns < ay.deadline_ns;
                return ax.criticality < ay.criticality;
            });
        v.erase(std::unique(v.begin(), v.end(),
            [](const auto& x, const auto& y){
                return std::get<0>(x)==std::get<0>(y) &&
                       std::get<1>(x)==std::get<1>(y) &&
                       same_ti(std::get<2>(x), std::get<2>(y));
            }), v.end());
    }
}

// Robust WCET lookup (same as before; keep yours if already added)
static double wcet_lookup_ms_for_state_method(
    int from_state,
    const std::string& method,
    const std::unordered_map<const void*, std::unordered_map<std::string,double>>& wcet_by_class)
{
    const void* cls_key = nullptr;
    if (auto itc = EnumToClassMap.find(from_state); itc != EnumToClassMap.end())
        cls_key = canon_key_from_record(itc->second);

    auto hit = wcet_by_class.find(cls_key);
    if (hit != wcet_by_class.end()) {
        auto fit = hit->second.find(method);
        if (fit != hit->second.end()) return fit->second;
    }
    hit = wcet_by_class.find(nullptr);
    if (hit != wcet_by_class.end()) {
        auto fit = hit->second.find(method);
        if (fit != hit->second.end()) return fit->second;
    }
    // last resort: scan all buckets
    for (const auto& b : wcet_by_class) {
        auto it = b.second.find(method);
        if (it != b.second.end()) return it->second;
    }
    return 0.0;
}

// End-to-end ASAP trace that stops on repeat
static void emit_end_to_end_asap_trace(
    const std::unordered_map<const void*, std::unordered_map<std::string,double>>& wcet_by_class)
{
    if (Timed_Typestate_Rules.empty()) return;

    // choose a start (prefer 0 if present)
    int start_state = Timed_Typestate_Rules.begin()->first;
    if (Timed_Typestate_Rules.count(0)) start_state = 0;

    std::fprintf(stderr, "[trace] --- end-to-end time trace ---\n");

    double t_ms = 0.0;
    int cur = start_state;

    // Stop criteria
    std::set<std::pair<int,std::string>> seen_edges; // (from_state, method)
    std::set<int> seen_states;                       // optional: track states too
    const int hard_cap = (int)Timed_Typestate_Rules.size() * 10;
  std::fprintf(stderr,
    "[trace] states=%zu, hard_cap=%d\n",
    Timed_Typestate_Rules.size(), hard_cap);

    for (int step = 0; step < hard_cap; ++step) {
        auto it = Timed_Typestate_Rules.find(cur);
        if (it == Timed_Typestate_Rules.end() || it->second.empty()) break;

        // deterministic: first outgoing edge after normalization
        const auto& edge   = it->second.front();
        const std::string& method   = std::get<0>(edge);
        const int          to_state = std::get<1>(edge);
        const TimedInfo&   ti       = std::get<2>(edge);

        // stop if we loop back to start after first hop
        if (step > 0 && cur == start_state) {
            std::fprintf(stderr, "[trace] stop: returned to start_state=%d\n", start_state);
            break;
        }
        // stop if this (state,method) already seen
        auto key = std::make_pair(cur, method);
        if (seen_edges.count(key)) {
            std::fprintf(stderr, "[trace] stop: cycle detected at state %d via '%s'\n",
                         cur, method.c_str());
            break;
        }
        seen_edges.insert(key);
        seen_states.insert(cur);

        const double wcet_ms = wcet_lookup_ms_for_state_method(cur, method, wcet_by_class);
        const double t_next  = t_ms + wcet_ms;

        std::fprintf(stderr,
    "[trace] t_%d=%.2f ms --%s()--> t_%d=%.2f ms\n",
    step, t_ms, method.c_str(), step + 1, t_next);


        if (ti.deadline_ns > 0) {
            std::fprintf(stderr, " (d<%.4f ms)", ns_to_ms(ti.deadline_ns));
        } else if (ti.period_ns > 0) {
            std::fprintf(stderr, " (p%s%.4f ms)", is_sporadic(ti) ? ">=" : "=", ns_to_ms(ti.period_ns));
        }
        std::fprintf(stderr, " [state %d -> %d]\n", cur, to_state);

        t_ms = t_next;
        cur = to_state;
    }
}




// ======================== FULL VALIDATOR ========================
static void validate_wcet_vs_timed_rules() {
    // 1) WCET source (prefer class-aware; else fallback)
    normalize_rules();
    std::unordered_map<const void*, std::unordered_map<std::string,double>> wcet_by_class = g_annot_wcet_ms;
    bool used_fallback = false;
    if (wcet_by_class.empty()) {
        wcet_by_class = build_wcet_map_fallback_from_rows();
        used_fallback = true;
    }

    if (kDebugValidate) {
        std::fprintf(stderr, "[validate] using %s for WCETs\n",
                     used_fallback ? "g_rows (fallback)" : "g_annot_wcet_ms");
        dump_wcet_buckets(wcet_by_class);
    }

    bool ok                  = true;  // overall success (warnings make this false)
    bool any_missing         = false; // missing buckets/entries (warning)
    bool any_violation       = false; // any violation (soft or hard)
    bool any_hard_violation  = false; // hard/firm violations (can fail build)

    // Keep a bucket pointer per originating state for the timeline phase
    std::unordered_map<int, const std::unordered_map<std::string,double>*> state_bucket;

    // For a global end-to-end trace across states
    struct EdgeSample {
        int from_state;
        int to_state;
        std::string method;
        double wcet_ms;
        TimedInfo ti;
        bool recursive;  // true if from_state == to_state
    };
    std::vector<EdgeSample> all_edges;

    double cycle_ms = 0.0;

    for (const auto& kv : Timed_Typestate_Rules) {
        int from_state = kv.first;
        const auto& edges = kv.second;

        // class key for this FSM state
        const void* cls_key = nullptr;
        if (auto itc = EnumToClassMap.find(from_state); itc != EnumToClassMap.end())
            cls_key = canon_key_from_record(itc->second);

        // exact class bucket or nullptr fallback
        auto bucket_it = wcet_by_class.find(cls_key);
        bool using_null_bucket = false;
        if (bucket_it == wcet_by_class.end()) {
            bucket_it = wcet_by_class.find(nullptr);
            using_null_bucket = (bucket_it != wcet_by_class.end());
        }

        if (bucket_it != wcet_by_class.end())
            state_bucket[from_state] = &bucket_it->second;
        else
            state_bucket[from_state] = nullptr;

        if (kDebugValidate) {
            std::fprintf(stderr, "[validate] state=%d class_key=%p  bucket=%s\n",
                         from_state, cls_key,
                         (bucket_it == wcet_by_class.end()) ? "NONE" :
                         (using_null_bucket ? "nullptr (fallback)" : "exact class"));
        }

        for (const auto& e : edges) {
            const std::string& method   = std::get<0>(e);
            const int          to_state = std::get<1>(e);
            const TimedInfo&   ti       = std::get<2>(e);

            const bool is_recursive = (from_state == to_state);
            if (is_recursive) {
                g_recursive_transitions.push_back({ method, from_state });
                g_recursive_method_transitions.insert(method);
                if (kDebugValidate) {
                    std::fprintf(stderr,
                        "[validate] detected recursive transition: state %d, method '%s'\n",
                        from_state, method.c_str());
                }
            }

            // Pick the per-transition budget (ms): deadline preferred; else non-sporadic period
            double rule_ms = 0.0;
            const char* rule_src = "none";
            const bool spor = is_sporadic(ti);
            const Contract ctr = contract_from_criticality(ti.criticality);

            if (ti.deadline_ns > 0) {
                rule_ms = ns_to_ms(ti.deadline_ns);
                rule_src = "deadline";
            } else if (ti.period_ns > 0 && !spor) {
                rule_ms = ns_to_ms(ti.period_ns);
                rule_src = "period-as-budget";
            } else {
                // No per-transition budget → skip compare, but still collect for timeline
                if (bucket_it != wcet_by_class.end()) {
                    const auto& bucket = bucket_it->second;
                    if (auto fn_it = bucket.find(method); fn_it != bucket.end()) {
                        const double wcet_ms = fn_it->second;
                        all_edges.push_back({from_state, to_state, method, wcet_ms, ti, is_recursive});
                    } else {
                        // report_violation(false,
                        //   "Missing WCET for '%s' (state %d). No deadline; sporadic or unspecified.",
                        //   method.c_str(), from_state);
                        any_missing = true; ok = false;
                    }
                } else {
                    report_violation(false,
                        "No WCETs recorded for class of state %d; missing '%s' (no deadline).",
                        from_state, method.c_str());
                    any_missing = true; ok = false;
                }
                continue;
            }

            if (bucket_it == wcet_by_class.end()) {
                // Missing class bucket → WARNING only
                report_violation(false,
                    "No WCETs recorded for class of state %d; missing '%s' (rule=%.6f ms).",
                    from_state, method.c_str(), rule_ms);
                if (kDebugValidate)
                    std::fprintf(stderr, "  [miss] %s: bucket missing (rule %.6f from %s)\n",
                                 method.c_str(), rule_ms, rule_src);
                any_missing = true; ok = false;
                continue;
            }

            const auto& bucket = bucket_it->second;
            auto fn_it = bucket.find(method);
            if (fn_it == bucket.end()) {
                // Missing method entry → WARNING only
                report_violation(false,
                    "Missing WCET for '%s' (state %d). Expected %.6f ms from %s.",
                    method.c_str(), from_state, rule_ms, rule_src);

                if (kDebugValidate) {
                    std::fprintf(stderr, "  [miss] %s: not in bucket. Have:", method.c_str());
                    int shown = 0;
                    for (const auto& kv2 : bucket) {
                        if (shown++ >= 12) { std::fprintf(stderr, " ..."); break; }
                        std::fprintf(stderr, " %s", kv2.first.c_str());
                    }
                    std::fprintf(stderr, "\n");
                }
                any_missing = true; ok = false;
                continue;
            }

            const double wcet_ms = fn_it->second;
            const double tol = std::max(kAbsTolMs, std::fabs(rule_ms) * kRelTol);

            bool hard_violate = false;
            bool soft_warn    = false;
            bool soft_over    = false;

            if (ctr == Contract::HARD) {
                // HARD/FIRM: WCET must not exceed the budget (within tolerance)
                hard_violate = (wcet_ms > rule_ms);
            } else {
                // SOFT: warn if near/equal or exceeding
                if (wcet_ms > rule_ms + tol) {
                    soft_warn = true; soft_over = true;
                } else if (wcet_ms >= rule_ms - tol) {
                    soft_warn = true;
                }
            }

            if (kDebugValidate) {
                std::fprintf(stderr,
                    "  [cmp] %-22s wcet=%.6f  rule=%.6f (%s)  tol=%.3g  (%s) -> %s\n",
                    method.c_str(), wcet_ms, rule_ms, rule_src, tol,
                    (ctr == Contract::HARD ? "HARD/FIRM" : "SOFT"),
                    (ctr == Contract::HARD ? (hard_violate ? "VIOLATION" : "ok")
                                           : (soft_warn ? (soft_over ? "warning(over)" : "warning")
                                                        : "ok")));
            }

            // Record for global path (raw wcet_ms; recursion handled later via effective WCET)
            all_edges.push_back({from_state, to_state, method, wcet_ms, ti, is_recursive});

            // Emit diagnostics
            if (ctr == Contract::HARD && hard_violate) {
                report_violation(true,
                    "WCET violates Time contract for '%s' (state %d): wcet=%.6f ms, "
                    "limit=%.6f ms (tol=%.3g).",
                    method.c_str(), from_state, wcet_ms, rule_ms, tol);
                any_violation = true; any_hard_violation = true; ok = false;
            } else if (ctr == Contract::SOFT && soft_warn) {
                if (soft_over) {
                    report_violation(false,
                        "WCET exceeds SOFT limit for '%s' (state %d): wcet=%.6f ms, "
                        "limit=%.6f ms (tol=%.3g).",
                        method.c_str(), from_state, wcet_ms, rule_ms, tol);
                } else {
                    report_violation(false,
                        "WCET near/equal to SOFT limit for '%s' (state %d): wcet=%.6f ms, "
                        "limit=%.6f ms (tol=%.3g).",
                        method.c_str(), from_state, wcet_ms, rule_ms, tol);
                }
                any_violation = true; ok = false;
            }
        }
    }

    // ---------------- System-level timing feasibility (windowed) ---------------
    for (const auto& kv2 : Timed_Typestate_Rules) {
        const int state = kv2.first;
        const auto& edges = kv2.second;

        static const std::unordered_map<std::string,double> kEmpty;
        const auto itB = state_bucket.find(state);
        const std::unordered_map<std::string,double>& bucket =
            (itB != state_bucket.end() && itB->second) ? *(itB->second) : kEmpty;

        run_timeline_for_state_ms(state, edges, bucket);
    }

    // --------- (optional) simple end-to-end trace across states ----------------
    if (kDebugValidate && !all_edges.empty()) {
        const int initial_state = 0; // adjust if your FSM uses another initial state
        std::vector<const EdgeSample*> path;
        std::vector<bool> used(all_edges.size(), false);

        // helper: effective WCET with recursion applied
        auto effective_wcet_ms = [](const EdgeSample& s) -> double {
            
            return s.recursive ? (depth_r * s.wcet_ms) : s.wcet_ms;
        };

        int current = initial_state;
        for (std::size_t step = 0; step < all_edges.size(); ++step) {
            std::size_t next_idx = all_edges.size();
            double max_eff_wcet = -1.0;

            // Pick the slowest (max *effective* WCET) outgoing edge from the current state
            for (std::size_t i = 0; i < all_edges.size(); ++i) {
                if (!used[i] && all_edges[i].from_state == current) {
                    double eff = effective_wcet_ms(all_edges[i]);
                    if (eff > max_eff_wcet) {
                        max_eff_wcet = eff;
                        next_idx = i;
                    }
                }
            }

            if (next_idx == all_edges.size())
                break; // no more outgoing edges from this state

            used[next_idx] = true;
            path.push_back(&all_edges[next_idx]);
            current = all_edges[next_idx].to_state;
        }

        if (!path.empty()) {
            std::fprintf(stderr, "[ttrace] --- end-to-end time trace ---\n");
            double t_ms = 0.0;

            for (std::size_t i = 0; i < path.size(); ++i) {
                const EdgeSample& s = *path[i];
                const TimedInfo& ti = s.ti;

                double rel_ms = ns_to_ms(worst_release_ns(ti));
                if (rel_ms > t_ms) t_ms = rel_ms;

                const double eff_wcet = effective_wcet_ms(s);
                if (s.recursive) {
                    std::printf("Recursive WCET applied for %s: %.4f ms\n",
                                s.method.c_str(), eff_wcet);
                }

                const double t_next = t_ms + eff_wcet;
                std::fprintf(stderr,
                    "[ttrace] t_%zu=%.4f ms --%s()%s--> t_%zu=%.4f ms",
                    i, t_ms,
                    s.method.c_str(),
                    s.recursive ? " [recursive]" : "",
                    i + 1, t_next);

                if (ti.deadline_ns > 0) {
                    std::fprintf(stderr, " (d<%.4f ms)", ns_to_ms(ti.deadline_ns));
                } else if (ti.period_ns > 0) {
                    std::fprintf(stderr, " (p%s%.4f ms)",
                                 is_sporadic(ti) ? ">=" : "=",
                                 ns_to_ms(ti.period_ns));
                }

                std::fprintf(stderr, " [state %d -> %d]\n", s.from_state, s.to_state);
                t_ms = t_next;

                if (i == path.size()-1){
                    std::fprintf(stderr,
                        "[ttrace] end at t_%zu=%.4f ms\n",
                        i + 1, t_ms);
                    cycle_ms = t_ms;
                }
            }
        }
    }
    // ---------------------------------------------------------------------------

    if (!ok) {
        if (kDebugValidate) {
            std::fprintf(stderr, "[validate] FAIL: %s%s%s\n",
                any_hard_violation ? "hard violations " : "",
                (any_violation && !any_hard_violation) ? "violations " : "",
                any_missing ? "(missing entries)" : "");
        }
        // Only hard-stop on HARD/FIRM violations
        if (kFailBuildOnViolation && any_hard_violation) {
            fatal_error(UNKNOWN_LOCATION,
                        "WCET vs Timed Typestate HARD/FIRM timing violations detected.");
        }
    } else {
        if (kDebugValidate) std::fprintf(stderr, "[validate] PASS: all checks ok\n");
    }

    if (cycle_ms > g_cycle_n) {
        error_at(UNKNOWN_LOCATION,
                 "End-to-end cycle time %.6f ms exceeds bound %.6f ms.",
                 cycle_ms, g_cycle_n);
    }

    emit_end_to_end_asap_trace(wcet_by_class);
}


void validate_LSP_compliance() {
    for (const auto& [subclass, sub_fsm] : FSM_by_Class) {
        tree sub_binfo = TYPE_BINFO(subclass);
        if (!sub_binfo) continue;

        for (int i = 0; i < BINFO_N_BASE_BINFOS(sub_binfo); ++i) {
            tree base_binfo = BINFO_BASE_BINFO(sub_binfo, i);
            tree base_class = BINFO_TYPE(base_binfo);

            if (FSM_by_Class.find(base_class) == FSM_by_Class.end()) continue;
            const auto& base_fsm = FSM_by_Class[base_class];

            for (const auto& [method, base_next_state] : base_fsm) {
                auto it = sub_fsm.find(method);
                if (it == sub_fsm.end()) {
                    fprintf(stderr, "[LSP Violation] Subclass '%s' missing method '%s' from base class.\n",
                            IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(subclass))), method.c_str());
                } else if (it->second != base_next_state) {
                    fprintf(stderr, "[LSP Violation] Subclass '%s' alters method '%s' transition. Expected %d, got %d.\n",
                            IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(subclass))), method.c_str(), base_next_state, it->second);
                }
            }
        }
    }
    
}


// Function to extract and store all template arguments of Flag
void process_TypestateClassConnector_args(tree tmpl_args) {
    int num_args = TREE_VEC_LENGTH(tmpl_args);

   
    TypestateClassConnector_args.clear();

   
    for (int i = 0; i < num_args; ++i) {
        tree arg = TREE_VEC_ELT(tmpl_args, i);

       
        if (TYPE_P(arg)) {
           
            tree type_decl = TYPE_NAME(arg);
            if (type_decl && TREE_CODE(type_decl) == TYPE_DECL) {
                const char* type_name = IDENTIFIER_POINTER(DECL_NAME(type_decl));
                if (type_name) {
                    TypestateClassConnector_args.push_back(type_name);  
                 
                } else {
               
                }
            } else {
               
            }
        } else if (TREE_CODE(arg) == INTEGER_CST) {
           
            HOST_WIDE_INT val = TREE_INT_CST_LOW(arg);
            TypestateClassConnector_args.push_back(std::to_string(val));  
   
        } else {
         
        }
    }

    if (TypestateClassConnector_args.empty()) {
        printf("Failed to extract any template arguments of TypestateClassConnectorFlag.\n");
    }
}
// Template instantiation processing

static void on_template_instantiation(void* gcc_data, void* user_data) {
    tree fn_decl = (tree)gcc_data; (void)user_data;
    if (!fn_decl || !DECL_TEMPLATE_INSTANTIATION(fn_decl)) return;

    if (TREE_CODE(fn_decl) == TYPE_DECL || TREE_CODE(fn_decl) == FUNCTION_DECL) {
        tree type_decl = DECL_CONTEXT(fn_decl);
        if (!type_decl || TREE_CODE(type_decl) != RECORD_TYPE) return;

        const char* decl_name = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(type_decl)));
        if (!decl_name) return;

        if (decl_name) {
    // 1) Existing direct cases (keep them, harmless if they fire)
    if (strcmp(decl_name, "State") == 0) {
        if (tree ta = DECL_TI_ARGS(fn_decl)) process_typestate_template_args(ta);
        return;
    }
    if (strcmp(decl_name, "Timed_State") == 0) {
        if (tree ta = DECL_TI_ARGS(fn_decl)) process_timed_typestate_args(ta);
        return;
    }
    if (strcmp(decl_name, "TypestateClassConnector") == 0) {
        if (tree ta = DECL_TI_ARGS(fn_decl)) process_TypestateClassConnector_args(ta);
        return;
    }

    // 2) The important case: the OUTER template that contains the pack
    if (strcmp(decl_name, "Typestate_Template") == 0) {
        if (tree pack_args = DECL_TI_ARGS(fn_decl)) {
            // Each element of this pack is State<...> or Timed_State<...>
            process_typestate_template_pack(pack_args);
        }
        return;
    }
}
    }
}

void Visualize_FSMs_Per_Class() {
    std::unordered_map<tree, std::map<int, std::vector<std::pair<std::string, int>>>> FSMs_by_class;

    for (const auto& [enum_val, transitions] : Typestate_Rules) {
        if (EnumToClassMap.find(enum_val) == EnumToClassMap.end()) continue;

        tree class_tree = EnumToClassMap[enum_val];
        FSMs_by_class[class_tree][enum_val] = transitions;
    }

    for (const auto& [class_tree, class_fsms] : FSMs_by_class) {
        if (!class_tree || !TYPE_NAME(class_tree)) continue;
        const char* class_name = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(class_tree)));
        if (!class_name) continue;

        std::string dot_file = std::string(class_name) + "_fsm.dot";
        std::string png_file = std::string(class_name) + "_fsm.png";

        std::ofstream dot(dot_file);
        dot << "digraph " << class_name << "_FSM {\n";
        dot << "  rankdir=LR;\n";
        dot << "  node [shape=circle];\n";

        for (const auto& [enum_val, transitions] : class_fsms) {
            for (const auto& [method, next_state] : transitions) {
                dot << "  " << enum_val << " -> " << next_state << " [label=\"" << method << "\"];\n";
            }
        }

        dot << "}\n";
        dot.close();

        std::string command = "dot -Tpng " + dot_file + " -o " + png_file;
        std::system(command.c_str());
        //std::cout << "[FSM] Graph for " << class_name << " written to " << png_file << "\n";
    }
}




static void Typestate_Visualisation(const std::string& filename,
                                    const std::string& imageFilename) {
    std::ofstream dotFile(filename);
    if (!dotFile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    auto fmt = [](long long v) { return v ? std::to_string(v) : std::string("-"); };

    auto escape_label = [](std::string s) {
        // Escape backslashes and quotes for DOT label strings
        std::string out;
        out.reserve(s.size() * 2);
        for (char c : s) {
            if (c == '\\' || c == '"') out.push_back('\\');
            out.push_back(c);
        }
        return out;
    };

    dotFile << "digraph StateMachine {\n";
    dotFile << "    rankdir=LR;\n";
    dotFile << "    node [shape=circle];\n";

    // Track emitted edges to avoid duplicates.
    // Key is (from, to, method)
    struct EdgeKey {
        int from{};
        int to{};
        std::string method;
        bool operator<(const EdgeKey& other) const {
            if (from != other.from) return from < other.from;
            if (to   != other.to)   return to   < other.to;
            return method < other.method;
        }
    };
    std::set<EdgeKey> emitted;

    // 1) Timed edges with multiline labels (deduped)
    for (const auto& [from, edges] : Timed_Typestate_Rules) {
        for (const auto& e : edges) {
            const std::string& method = std::get<0>(e);
            int to                    = std::get<1>(e);
            const TimedInfo& ti       = std::get<2>(e);

            EdgeKey key{from, to, method};
            if (emitted.count(key)) continue; // already emitted

            emitted.insert(key);

            std::ostringstream lbl;
            lbl << method
               // << "\\nR=" << fmt(ti.release_ns)
                << " P="  << fmt(ti.period_ns)
                << " D="  << fmt(ti.deadline_ns)
                << " Offset=" << fmt(ti.jitter_ns)
                << " C="  << ti.criticality;

            dotFile << "    " << from << " -> " << to << " "
                    << "[label=\"" << escape_label(lbl.str()) << "\"];\n";
        }
    }

    // 2) Untimed edges only if not already emitted (deduped)
    for (const auto& [from, transitions] : Typestate_Rules) {
        for (const auto& [method, to] : transitions) {
            EdgeKey key{from, to, method};
            if (emitted.count(key)) continue; // a timed or prior untimed already covered it
            emitted.insert(key);

            dotFile << "    " << from << " -> " << to
                    << " [label=\"" << escape_label(method) << "\"];\n";
        }
    }

    dotFile << "}\n";
    dotFile.close();

    std::cout << "Graphviz .dot file generated: " << filename << std::endl;

    const std::string command = "dot -Tpng " + filename + " -o " + imageFilename;
    const int result = std::system(command.c_str());
    if (result == 0) {
        std::cout << "Graphviz image generated: " << imageFilename << std::endl;
    } else {
        std::cerr << "Error generating image. Ensure Graphviz is installed and available in PATH.\n";
    }
}




   void merge_typestate_rules_carefully() {
    std::map<int, std::vector<std::pair<std::string, int>>> Merged_Typestate_Rules;

    // Iterate over all entries in Typestate_Rules
    for (const auto& [enum_value, main_transitions] : Typestate_Rules) {
        // Check if the same enum_value exists in Subtype_Typestate_Rules
        if (Subtype_Typestate_Rules.find(enum_value) != Subtype_Typestate_Rules.end()) {
            const auto& subtype_transitions = Subtype_Typestate_Rules[enum_value];
            std::vector<std::pair<std::string, int>> merged_transitions;

            // Find the merge point (common prefix of transitions)
            size_t merge_point = 0;
            while (merge_point < main_transitions.size() && merge_point < subtype_transitions.size() &&
                   main_transitions[merge_point] == subtype_transitions[merge_point]) {
                merged_transitions.push_back(main_transitions[merge_point]);
                merge_point++;
            }

            // Add the remaining transitions separately after the merge point
            merged_transitions.insert(merged_transitions.end(),
                                      main_transitions.begin() + merge_point, main_transitions.end());
            merged_transitions.insert(merged_transitions.end(),
                                      subtype_transitions.begin() + merge_point, subtype_transitions.end());

            Merged_Typestate_Rules[enum_value] = merged_transitions;
        } else {
            // If no matching enum_value in Subtype_Typestate_Rules, just copy main transitions
            Merged_Typestate_Rules[enum_value] = main_transitions;
        }
    }

    // Add any remaining entries from Subtype_Typestate_Rules that are not in Typestate_Rules
    for (const auto& [enum_value, subtype_transitions] : Subtype_Typestate_Rules) {
        if (Merged_Typestate_Rules.find(enum_value) == Merged_Typestate_Rules.end()) {
            Merged_Typestate_Rules[enum_value] = subtype_transitions;
        }
    }

    // Overwrite Typestate_Rules with the merged result
    Typestate_Rules = Merged_Typestate_Rules;

    // Clear Subtype_Typestate_Rules (optional)
    Subtype_Typestate_Rules.clear();

    // Print the final merged typestate rules
    printf("\n--- Final Typestate Rules ---\n");
    for (const auto& [enum_value, transitions] : Typestate_Rules) {
        printf("Enum value: %d\n", enum_value);
        for (const auto& [function_name, next_state] : transitions) {
            printf("  Function: %s -> Next state: %d\n", function_name.c_str(), next_state);
        }
    }
}

    void ProcessSuperSubStates() {
   
    std::unordered_map<int, std::unordered_map<std::string, int>> state_tracker;
    int counter = 0;

   
    std::map<int, std::vector<std::pair<std::string, int>>> processed_states;

     for (const auto& [key, states] : Typestate_Rules) {
        std::vector<std::pair<std::string, int>> super_sub_states;

        for (const auto& [state, value] : states) {
            if (state_tracker[key].find(state) == state_tracker[key].end()) {
                // Superstate
                state_tracker[key][state] = ++counter;
                super_sub_states.emplace_back(state + "-superstate", state_tracker[key][state]);
                std::cout<<state<<"is superstate"<<"\n";
            } else {
            // Substate
                super_sub_states.emplace_back(state + "-substate", state_tracker[key][state]);
                std::cout<<state<< "is substate"<<"\n";
            }
        }

       
        processed_states[key] = super_sub_states;
    }

}




void print_typestate_rules() {
    printf("\n Typestate Rules \n");
    for (const auto& [enum_value, transitions] : Typestate_Rules) {
        printf("Enum value: %d\n", enum_value);
        for (const auto& [function_name, next_state] : transitions) {
            printf("  Function: %s -> Next state: %d\n", function_name.c_str(), next_state);
        }
    }

    printf("\n Subtype Typestate Rules \n");
    for (const auto& [enum_value, transitions] : Subtype_Typestate_Rules) {
        printf("Enum value: %d\n", enum_value);
        for (const auto& [function_name, next_state] : transitions) {
            printf("  Function: %s -> Next state: %d\n", function_name.c_str(), next_state);
        }
    }
}

bool leads_to(
    const std::map<int, std::vector<std::pair<std::string, int>>>& fsm,
    int from,
    int expected_dest,
    std::unordered_set<int>& visited
) {
    if (from == expected_dest) return true;
    if (visited.count(from)) return false;
    visited.insert(from);

    auto it = fsm.find(from);
    if (it == fsm.end()) return false;

    for (const auto& [_, next] : it->second) {
        if (leads_to(fsm, next, expected_dest, visited)) {
            return true;
        }
    }

    return false;
}

bool leads_to(
    const std::map<int, std::vector<std::pair<std::string, int>>>& fsm,
    int from,
    int expected_dest
) {
    std::unordered_set<int> visited;
    return leads_to(fsm, from, expected_dest, visited);
}
bool is_reachable_from_base_start(
    const std::map<int, std::vector<std::pair<std::string, int>>>& fsm,
    int base_start_state,
    int target
) {
    std::unordered_set<int> visited;
    return leads_to(fsm, base_start_state, target, visited);
}

bool method_reused_with_different_transition(
    const std::string& method,
    int start_state,
    int next_state,
    const std::map<int, std::vector<std::pair<std::string, int>>>& base_fsm
) {
    for (const auto& [base_src, base_transitions] : base_fsm) {
        for (const auto& [base_method, base_dst] : base_transitions) {
            if (base_method == method) {
                if (base_src != start_state || base_dst != next_state) {
                    return true;  // found a conflict
                }
            }
        }
    }
    return false;
}


bool merge_typestate_fsm_respecting_lsp(
    const std::string& subclass_name,
    const std::string& base_name,
    const std::map<int, std::vector<std::pair<std::string, int>>>& base_fsm,
    const std::map<int, std::vector<std::pair<std::string, int>>>& subclass_fsm,
    std::map<int, std::vector<std::pair<std::string, int>>>& merged_fsm
) {
    merged_fsm = base_fsm;

    // Collect all method names used in base FSM
    std::set<std::string> base_method_names;
    for (const auto& [_, transitions] : base_fsm) {
        for (const auto& [method, __] : transitions) {
            base_method_names.insert(method);
        }
    }

   

    for (const auto& [start_state, subclass_transitions] : subclass_fsm) {
        for (const auto& [method, next_state] : subclass_transitions) {

            // Detect reuse of any base method name from a different state
            if (base_method_names.count(method)) {
                bool identical_transition_found = false;
                for (const auto& [base_state, base_transitions] : base_fsm) {
                    for (const auto& [base_method, base_next] : base_transitions) {
                        if (base_method == method && base_state == start_state && base_next == next_state) {
                            identical_transition_found = true;
                            break;
                        }
                    }
                    if (identical_transition_found) break;
                }

                if (!identical_transition_found) {
                    error_at(UNKNOWN_LOCATION,
                        "Typestate LSP violation: method '%s' from base FSM is reused in subclass '%s' "
                        "with a different transition (subclass: %d → %d). Reusing method names with different semantics is not allowed.",
                        method.c_str(), subclass_name.c_str(),
                        start_state, next_state);
                    return false;
                }
            }

            bool method_in_base = false;
            bool allowed = false;

            if (method_reused_with_different_transition(method, start_state, next_state, base_fsm)) {
    error_at(UNKNOWN_LOCATION,
        "Typestate LSP violation: method '%s' reused in subclass '%s' with a different transition "
        "(subclass: %d → %d).",
        method.c_str(), subclass_name.c_str(),
        start_state, next_state);
    return false;
}


            // Case 1: method exists at this start state in base
            if (base_fsm.count(start_state)) {
                for (const auto& [base_method, base_next_state] : base_fsm.at(start_state)) {
                    if (base_method == method) {
                        method_in_base = true;

                        if (leads_to(subclass_fsm, next_state, base_next_state)) {
                            allowed = true;
                            break;
                        } else {
                            error_at(UNKNOWN_LOCATION,
                                "Typestate LSP violation: subclass '%s' redirects method '%s' from state %d "
                                "to incompatible state %d (base expects %d).",
                                subclass_name.c_str(), method.c_str(),
                                start_state, next_state, base_next_state);
                            return false;
                        }
                    }
                }

                // Case 2: new method at a known state — allow if leads to base-valid state
                if (!method_in_base) {
                    for (const auto& [_, base_next_state] : base_fsm.at(start_state)) {
                        if (leads_to(subclass_fsm, next_state, base_next_state)) {
                            allowed = true;
                            break;
                        }
                    }
                }
            }

            // Case 3: method added from unknown state — must be reachable
            if (!method_in_base && !allowed) {
                bool known_start = merged_fsm.count(start_state);

                if (!known_start) {
                    for (const auto& [_, transitions] : merged_fsm) {
                        for (const auto& [_, dst] : transitions) {
                            if (leads_to(subclass_fsm, dst, start_state)) {
                                known_start = true;
                                break;
                            }
                        }
                        if (known_start) break;
                    }
                }

                if (!known_start) {
                    error_at(UNKNOWN_LOCATION,
                        "Typestate LSP violation: subclass '%s' introduces method '%s' from state %d, "
                        "which is not reachable from any known base or merged FSM state.",
                        subclass_name.c_str(), method.c_str(), start_state);
                    return false;
                }

                allowed = true;
            }

            // Merge transition if allowed
            if (allowed) {
                merged_fsm[start_state].emplace_back(method, next_state);
            }
        }
    }

    return true;
}




bool try_merge_fsms_respecting_lsp_for_class_pair() {
    static std::map<std::string, std::map<int, std::vector<std::pair<std::string, int>>>> FSMs_by_class;

    // Step 1: Build FSMs_by_class safely
    for (const auto& [enum_val, transitions] : Typestate_Rules) {
        if (EnumToClassMap.find(enum_val) == EnumToClassMap.end()) continue;

        tree class_tree = EnumToClassMap[enum_val];
        if (!class_tree || TREE_CODE(class_tree) != RECORD_TYPE) continue;

        const char* cname = nullptr;
        if (TYPE_NAME(class_tree) && DECL_NAME(TYPE_NAME(class_tree))) {
            cname = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(class_tree)));
        }
        if (!cname) continue;

        FSMs_by_class[cname][enum_val] = transitions;
    }

    // Step 2: Pair subclass/baseclass combinations and try merge
    for (const auto& [e_sub, subclass_tree] : EnumToClassMap) {
        if (!subclass_tree || TREE_CODE(subclass_tree) != RECORD_TYPE) continue;

        for (const auto& [e_base, base_tree] : EnumToClassMap) {
            if (!base_tree || TREE_CODE(base_tree) != RECORD_TYPE) continue;
            if (subclass_tree == base_tree) continue;
            if (!is_subclass(subclass_tree, base_tree)) continue;

            const char* sub_cname = nullptr;
            const char* base_cname = nullptr;

            if (TYPE_NAME(subclass_tree) && DECL_NAME(TYPE_NAME(subclass_tree)))
                sub_cname = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(subclass_tree)));
            if (TYPE_NAME(base_tree) && DECL_NAME(TYPE_NAME(base_tree)))
                base_cname = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(base_tree)));

            if (!sub_cname || !base_cname) continue;

            // Look up FSMs
            if (FSMs_by_class.find(base_cname) == FSMs_by_class.end() ||
                FSMs_by_class.find(sub_cname) == FSMs_by_class.end())
                continue;

            const auto& base_fsm = FSMs_by_class[base_cname];
            const auto& subclass_fsm = FSMs_by_class[sub_cname];

            std::map<int, std::vector<std::pair<std::string, int>>> merged_result;
            merge_typestate_fsm_respecting_lsp(
                sub_cname, base_cname, base_fsm, subclass_fsm, merged_result
            );
        }
    }

    return true;
}


// Callback function to be called at the end of compilation
static void on_finish(void* gcc_data, void* user_data) {
   
 (void)gcc_data;
 (void)user_data;
   
    if (TypestateClassConnector_args.empty()) {
      printf("Flag template was not instantiated or its template arguments were not extracted.\n");
    }

std::string base_name = TypestateClassConnector_args[0];
std::string dot_file = base_name + "_state_machine.dot";
std::string png_file = base_name + "_state_machine.png";

Typestate_Visualisation(dot_file.c_str(), png_file.c_str());
// LSP Subclass FSM Validation
for (const auto& [sub_enum, sub_class] : EnumToClassMap) {
    if (!sub_class || TREE_CODE(sub_class) != RECORD_TYPE) continue;

    for (const auto& [super_enum, super_class] : EnumToClassMap) {
        if (!super_class || TREE_CODE(super_class) != RECORD_TYPE || sub_class == super_class) continue;

        if (is_subclass(sub_class, super_class)) {
            const auto& sub_fsm = FSM_by_Class[sub_class];
            const auto& super_fsm = FSM_by_Class[super_class];

            for (const auto& [method, sub_target] : sub_fsm) {
                auto it = super_fsm.find(method);
                if (it != super_fsm.end()) {
                    int super_target = it->second;
                    if (sub_target != super_target) {
                        const char* sub_class_name = "<unknown>";
                        const char* super_class_name = "<unknown>";

                        if (TYPE_NAME(sub_class) && DECL_NAME(TYPE_NAME(sub_class)))
                            sub_class_name = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(sub_class)));
                        if (TYPE_NAME(super_class) && DECL_NAME(TYPE_NAME(super_class)))
                            super_class_name = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(super_class)));

                        warning(0, "Typestate LSP violation: method '%s' in subclass '%s' transitions to %d, but in superclass '%s' it transitions to %d.",
                            method.c_str(), sub_class_name, sub_target, super_class_name, super_target);
                    }
                }
            }
        }
    }
    //validate_LSP_compliance();
 
 //   print_annotated_wcet_list();     // 1) show the collected list
   
     //Visualize_FSMs_Per_Class();
}

//print_typestate_rules();
//merge_typestate_rules_carefully();

}




/**
 Methods to Resolve Object References and Aliases:
   - `get_original_object` function simplifies complex references into their base forms, handling various cases
   like dereferencing and address calculations.
   - `track_all_aliases` function manages to track aliases across different scopes using
   local and global alias maps and resolves them to their original addresses.
**/

tree get_original_object(tree arg) {
    if (!arg) return arg;

    while (true) {
        enum tree_code code = TREE_CODE(arg);

        if (code == SSA_NAME) {
            // Resolve SSA_NAME to its underlying variable
            tree base_var = SSA_NAME_VAR(arg);
            if (base_var) {
                arg = base_var;
            } else {
                break;
            }
        } else if (code == MEM_REF || code == COMPONENT_REF || code == INDIRECT_REF || code == ARRAY_REF) {
            // Handle member access, pointer dereference, and array access
            arg = TREE_OPERAND(arg, 0);
        } else if (code == NOP_EXPR || code == CONVERT_EXPR || code == NON_LVALUE_EXPR) {

            arg = TREE_OPERAND(arg, 0);
        } else if (code == ADDR_EXPR) {
            // Address-of operator
            arg = TREE_OPERAND(arg, 0);
        } else {
            break;
        }
        if (!arg) {
            break;
        }
    }

    return arg;

}



tree track_all_aliases(tree obj_address) {
    if (!obj_address) return nullptr;


    std::unordered_set<tree> visited;

    // Resolve references first
    obj_address = get_original_object(obj_address);
    auto resolve_with_map = [&](std::unordered_map<tree, tree>& alias_map, const char* map_name) {
        while (alias_map.find(obj_address) != alias_map.end()) {
            if (visited.find(obj_address) != visited.end()) {
              (void)map_name;
                break;
            }
            visited.insert(obj_address);
            obj_address = alias_map[obj_address];
        }
    };

    // Check alias maps in the call stack
    for (auto& frame : call_stack) {
        resolve_with_map(frame.alias_map, "call_stack");
    }

    // Check the global alias map
    resolve_with_map(global_alias_map, "global_alias_map");
    return obj_address;
}


/**
 *
 * State_Manager Class
 * This class is central to managing the state of objects within the program.
 * The class also logs state changes, validates method calls against the current state of an object,
 * and manages transitions to new states as dictated by the info extracted from `typestate` template.
 * This is crucial for enforcing correct program behavior and debugging complex state-dependent interactions in the software.
   
 **/


class State_Manager {
public:
    // Register an alias for an object
    void register_alias(tree alias_address, tree original_address) {
    if (!alias_address || !original_address) return;


    original_address = track_all_aliases(original_address);
    call_stack.back().alias_map[alias_address] = original_address;

}

    // Register a method call and enforce the state transitions
 void Typestate_Checking (tree obj, const std::string& method_name, location_t location) {
    // Resolve the reference to get the base object
    tree obj_address = get_original_object(obj);

   
    tree resolved_obj = track_all_aliases(obj_address);

    if (!resolved_obj) {

        return;
    }

    // Retrieve the current state of the object
    if (object_states.find(resolved_obj) == object_states.end()) {
        object_states[resolved_obj] = 0;  
    }

    int current_state = object_states[resolved_obj];

    // Check if the method is part of any state transitions
    bool is_transition_method = false;
    for (const auto& state_entry : Typestate_Rules) {
        for (const auto& trans : state_entry.second) {
            if (trans.first == method_name) {
                is_transition_method = true;
                break;
            }
        }
        if (is_transition_method) break;
    }

    if (!is_transition_method) {
       
        method_calls[resolved_obj].push_back(method_name);
        return;
    }

   
    int next_state = -1;
    if (Typestate_Rules.find(current_state) != Typestate_Rules.end()) {
        for (const auto& trans : Typestate_Rules[current_state]) {
            if (trans.first == method_name) {
                next_state = trans.second;  
                break;
            }
        }
    }
     
   

    if (next_state != -1) {
       
        object_states[resolved_obj] = next_state;
     printf("Transition: Method '%s' caused the state change from %d to %d.",
        method_name.c_str(), current_state, next_state);
   
    } else {
   
     error_at(location, "Error: Method '%s' is not allowed in the current state %d.",
              method_name.c_str(), current_state);
         

    }

    method_calls[resolved_obj].push_back(method_name);
}


   
   tree track_all_aliases(tree obj_address) {
    if (!obj_address) return nullptr;

   
    for (auto& frame : call_stack) {
        if (frame.alias_map.find(obj_address) != frame.alias_map.end()) {
            obj_address = frame.alias_map[obj_address];
        }
    }

    std::unordered_set<tree> visited;
    while (global_alias_map.find(obj_address) != global_alias_map.end()) {
        if (visited.find(obj_address) != visited.end()) {
            printf("Alias resolution detected a cycle; breaking.\n");
            break;
        }
        visited.insert(obj_address);
        obj_address = global_alias_map[obj_address];
    }

    return obj_address;
}

    void print_state() {
        for (const auto& entry : object_states) {
            printf("  Object %p is in state %d\n", (void*)entry.first, entry.second);
        }

        for (const auto& entry : method_calls) {
            printf("  Object %p called methods: ", (void*)entry.first);
            for (const auto& method : entry.second) {
                printf("%s ", method.c_str());
            }
           printf("\n");
        }
    }


      std::unordered_map<tree, int> object_states;      
       std::unordered_map<tree, std::vector<std::string>> method_calls;  
private:
    std::unordered_map<tree, tree> global_alias_map;  
   
};

State_Manager state_manager;


/**
 * methods are used for analysing typestate transitions in branching
 * control-flow constructs (e.g., if-else or switch) within a program.
 * detect common joint states for all branches,
 * and once the branches are completed, the object is moved to the first common jintstate.
 * ensure that each method call inside the branches adheres to the specified typestate rules.
 *
 */
// Print incoming-edge counts for every state, then return the "joint" set (incoming > 1).
static std::set<int>
findCommonJointStatesAndPrint(const std::map<int, std::vector<std::pair<std::string,int>>>& rules) {
    std::unordered_map<int,int> incoming_count;

    for (const auto& [src, vec] : rules) {
        (void)src;
        for (const auto& [name, dst] : vec) {
            (void)name;
            incoming_count[dst]++;
        }
    }

    fprintf(stderr, " Typestate incoming-edge counts \n");
    for (const auto& [state, cnt] : incoming_count) {
        fprintf(stderr, "  state %d <- %d incoming\n", state, cnt);
    }

    std::set<int> joint_states;
    for (const auto& [state, cnt] : incoming_count) {
        if (cnt > 1) joint_states.insert(state);
    }

    fprintf(stderr, "Joint states (>1 incoming): {");
    bool first = true;
    for (int s : joint_states) { fprintf(stderr, "%s%d", first?"":", ", s); first = false; }
    fprintf(stderr, "}\n");
    return joint_states;
}


int get_if_else_id_for_condition(gimple* cond_stmt) {
    auto it = condition_id_map.find(cond_stmt);
    if (it != condition_id_map.end()) {
        return it->second;
    } else {
        int id = next_if_else_id++;
        condition_id_map[cond_stmt] = id;
   
        if_else_context_map[id] = {3, 0};
        if_else_status[id] = false;
        return id;
    }

     
}


std::unordered_map<gimple*, int> if_else_id_map;  // Maps conditionals to IDs
std::unordered_map<basic_block, int> bb_to_if_else_id;  // Maps BBs to IDs
int if_else_counter = 0;

gimple* get_controlling_cond(basic_block bb) {
    edge e;
    edge_iterator ei;
    FOR_EACH_EDGE(e, ei, bb->preds) {
        basic_block pred = e->src;
        gimple_stmt_iterator gsi = gsi_last_bb(pred);
        if (gsi_end_p(gsi))
            continue;
        gimple* stmt = gsi_stmt(gsi);
        if (gimple_code(stmt) == GIMPLE_COND)
            return stmt;
    }
    return nullptr;
}



bool is_inside_branch(basic_block bb) {
    if (!bb) return false;

    edge e;
    edge_iterator ei;
    FOR_EACH_EDGE(e, ei, bb->preds) {
        basic_block pred_bb = e->src;
        for (gimple_stmt_iterator gsi = gsi_start_bb(pred_bb); !gsi_end_p(gsi); gsi_next(&gsi)) {
            gimple* stmt = gsi_stmt(gsi);
            if (gimple_code(stmt) == GIMPLE_COND) {
                // Found a conditional statement
                
                printf("Debug: Entering conditional branch in basic block %p.\n", (void*)bb);
                return true;
            }
            if (gimple_code(stmt) == GIMPLE_SWITCH) {
                // Found a `switch` conditional statement
            // printf("Debug: Entering switch branch in basic block %p.\n", (void*)bb);
                return true;
            }
        }
    }
    return false;
}

int finalize_if_else_block(int if_else_id) {
    if (if_else_status[if_else_id]) {
        // Compute joint states now that all branches are complete
        std::set<int> detected_joint_states = findCommonJointStatesAndPrint(Typestate_Rules);

        // Update the global joint_states set
        for (int st : detected_joint_states) {
            joint_states.insert(st);
        }

        int first_joint_state = -1;
        if (!detected_joint_states.empty()) {
            first_joint_state = *detected_joint_states.begin();
        }

       // printf("Debug: Finalizing if-else block %d with %d joint states.\n", if_else_id, detected_joint_states.size());

        last_finalized_if_else_id = if_else_id;
        last_finalized_first_state = first_joint_state;

        current_if_else_id++;
        return first_joint_state;
    } else {
       //printf("Debug: If-else block %d is incomplete.\n", if_else_id);
        return -1;
    }
}

#include "gcc-plugin.h"
#include "basic-block.h"
#include "gimple.h"
#include "gimple-iterator.h"
#include "cfgloop.h"
#include "cfgloop.h"
#include "cfgloopmanip.h"

// 1) Normalize names (rules may be unqualified; analysis often demangles qualified)
static inline std::string unqualified_name(const std::string& demangled) {
    auto pos = demangled.rfind("::");
    return (pos == std::string::npos) ? demangled : demangled.substr(pos + 2);
}

// 2) Rule lookups that accept both qualified and unqualified names
#include <unordered_set>
#include <cstdio>



static inline std::pair<int,int> pick_two_arms(const std::vector<int>& cand) {
    int t = -1, f = -1;
    for (int s : cand) {
        if (t < 0) t = s;
        else if (f < 0 && s != t) { f = s; break; }
    }
    if (t >= 0 && f < 0) f = t; // single unique candidate → both arms same
    return {t, f};
}

// Works for plain bool/int literals (INTEGER_CST) and enum constants (CONST_DECL -> INTEGER_CST).
static bool is_falsey_const(tree t) {
    if (!t) return false;
    if (TREE_CODE(t) == INTEGER_CST) return integer_zerop(t);
    if (TREE_CODE(t) == CONST_DECL) {
        tree val = DECL_INITIAL(t);
        return val && TREE_CODE(val) == INTEGER_CST && integer_zerop(val);
    }
    return false;
}

static bool is_truthy_const(tree t) {
    if (!t) return false;
    if (TREE_CODE(t) == INTEGER_CST) return !integer_zerop(t);
    if (TREE_CODE(t) == CONST_DECL) {
        tree val = DECL_INITIAL(t);
        return val && TREE_CODE(val) == INTEGER_CST && !integer_zerop(val);
    }
    return false;
}



static bool ssa_derives_from(tree a, tree b, int depth = 6) {
    if (!a || !b) return false;
    if (a == b) return true;
    if (TREE_CODE(a) != SSA_NAME || depth <= 0) return false;
    gimple* def = SSA_NAME_DEF_STMT(a);
    if (!def) return false;

    if (is_gimple_call(def)) return gimple_call_lhs(def) == b;

    if (is_gimple_assign(def)) {
        if (gimple_num_ops(def) >= 2) {
            tree r1 = gimple_assign_rhs1(def);
            if (r1 == b || (TREE_CODE(r1)==SSA_NAME && ssa_derives_from(r1,b,depth-1))) return true;
        }
        if (gimple_num_ops(def) >= 3) {
            tree r2 = gimple_assign_rhs2(def);
            if (r2 == b || (TREE_CODE(r2)==SSA_NAME && ssa_derives_from(r2,b,depth-1))) return true;
        }
        if (gimple_num_ops(def) >= 4) {
            tree r3 = gimple_assign_rhs3(def);
            if (r3 == b || (TREE_CODE(r3)==SSA_NAME && ssa_derives_from(r3,b,depth-1))) return true;
        }
    }
    return false;
}

// Follow SSA back to the defining call (up to small depth)
static gimple* ssa_def_call(tree ssa, int depth = 6) {
    if (!ssa || depth <= 0 || TREE_CODE(ssa) != SSA_NAME) return nullptr;
    gimple* def = SSA_NAME_DEF_STMT(ssa);
    if (!def) return nullptr;
    if (is_gimple_call(def)) return def;
    if (is_gimple_assign(def)) {
        for (int i = 1; i <= 3 && gimple_num_ops(def) >= i+1; ++i) {
            tree r = (i==1)? gimple_assign_rhs1(def)
                   : (i==2)? gimple_assign_rhs2(def)
                           : gimple_assign_rhs3(def);
            if (TREE_CODE(r) == SSA_NAME)
                if (auto* c = ssa_def_call(r, depth-1)) return c;
        }
    }
    return nullptr;
}

// Given a GIMPLE_COND, recover the *call* whose result it tests.
static gimple* find_guard_call_for_cond(gimple* term) {
    if (!term || gimple_code(term) != GIMPLE_COND) return nullptr;
    tree L = gimple_cond_lhs(term), R = gimple_cond_rhs(term);
    if (TREE_CODE(L) == SSA_NAME) if (auto* c = ssa_def_call(L)) return c;
    if (TREE_CODE(R) == SSA_NAME) if (auto* c = ssa_def_call(R)) return c;

    // Fallback: scan the terminator’s block
    basic_block bb = gimple_bb(term);
    for (gimple_stmt_iterator gsi = gsi_start_bb(bb); !gsi_end_p(gsi); gsi_next(&gsi)) {
        gimple* s = gsi_stmt(gsi);
        if (!is_gimple_call(s)) continue;
        tree res = gimple_call_lhs(s);
        if (!res || TREE_CODE(res) != SSA_NAME) continue;
        bool lhs_from = (L == res) || (TREE_CODE(L)==SSA_NAME && ssa_derives_from(L,res));
        bool rhs_from = (R == res) || (TREE_CODE(R)==SSA_NAME && ssa_derives_from(R,res));
        if (lhs_from || rhs_from) return s;
    }
    return nullptr;
}



// Does the COND compare the call result 'res' to a constant K ?

// Is the COND effectively "if (res)" or "if (!res)

// Does this call's result feed a GIMPLE_SWITCH index?
static bool call_feeds_switch(gimple* call_stmt, gimple* term) {
    if (!call_stmt || !term || gimple_code(term) != GIMPLE_SWITCH) return false;
    tree res = gimple_call_lhs(call_stmt);
    if (!res || TREE_CODE(res) != SSA_NAME) return false;

    const gswitch* gs = as_a<const gswitch*>(term);
    tree idx = gimple_switch_index(gs);
    if (!idx) return false;

    if (idx == res) return true;
    if (TREE_CODE(idx) == SSA_NAME && ssa_derives_from(idx, res)) return true;
    return false;
}

// Extract (edge, case-value) pairs. Treat default separately in your code.


// Does the TRUE edge of 'term' correspond to "call_result == true"?
static bool true_edge_means_call_true(gimple* term, gimple* call_stmt) {
    if (!term || gimple_code(term) != GIMPLE_COND || !call_stmt) return true; // default
    tree res = gimple_call_lhs(call_stmt);
    if (!res || TREE_CODE(res) != SSA_NAME) return true;

    enum tree_code code = gimple_cond_code(term); // EQ_EXPR / NE_EXPR / etc.
    tree lhs = gimple_cond_lhs(term);
    tree rhs = gimple_cond_rhs(term);

    bool lhs_is_call = (lhs == res) || (TREE_CODE(lhs)==SSA_NAME && ssa_derives_from(lhs,res));
    bool rhs_is_call = (rhs == res) || (TREE_CODE(rhs)==SSA_NAME && ssa_derives_from(rhs,res));
    if (!lhs_is_call && !rhs_is_call) return true; // default

    tree other = lhs_is_call ? rhs : lhs;

    if (code == EQ_EXPR || code == NE_EXPR) {
        bool other_falsey = is_falsey_const(other);
        bool other_truthy = !other_falsey;

        // TRUE edge for EQ means (call == other); for NE means (call != other).
        // If 'other' is truthy → EQ: TRUE means call==true; NE: TRUE means call==false.
        // If 'other' is falsey → EQ: TRUE means call==false; NE: TRUE means call==true.
        if (code == EQ_EXPR) return other_truthy;
        else                 return !other_truthy;
    }

  
    return true;
}

static std::vector<int> transitions_for(int src, const std::string& method,  location_t loc = UNKNOWN_LOCATION) {
    std::vector<int> out;
    //findCommonJointStatesAndPrint(Typestate_Rules); // for debug visibility
    auto it = Typestate_Rules.find(src);
    if (it == Typestate_Rules.end()) {
    //  fprintf(stderr, "Debug: transitions_for: no rule entry for src=%d (method='%s')\n",
           //   src, method.c_str());
                error_at(loc, "Typestate: no rule entry for state %d and method '%s'.",
                src, method.c_str()); 
        return out;
    }

    const std::string uq = unqualified_name(method);
    std::unordered_set<int> seen;
    int total_considered = 0;
    int total_matched = 0;

    // Print the rule row we’re about to scan
  //fprintf(stderr, "Debug: transitions_for: scanning rules for src=%d (method='%s', uq='%s'):\n",
      //    src, method.c_str(), uq.c_str());

    for (const auto& kv : it->second) {
        const std::string& name = kv.first;
        int dst = kv.second;
        ++total_considered;

        bool name_match = (name == method) || (name == uq);

        // Per-entry trace
     // fprintf(stderr, "  - rule[%d]: name='%s' dst=%d%s\n",
           //   total_considered, name.c_str(), dst, name_match ? "  <-- match" : "");

        if (!name_match) continue;

        ++total_matched;
        if (!seen.count(dst)) {
            seen.insert(dst);
            out.push_back(dst);
        } else {
            // Duplicate destination suppressed
        //  fprintf(stderr, "    (duplicate dst=%d suppressed)\n", dst);
        }
    }

    // Summary
    if (out.empty()) {
     // fprintf(stderr, "Debug: transitions_for: NO transitions for src=%d (method='%s'). "
            //          "considered=%d matched=%d\n",
          //    src, method.c_str(), total_considered, total_matched);
    } else {
      //fprintf(stderr, "Debug: transitions_for: RESULT for src=%d (method='%s'): {",
         //     src, method.c_str());
        for (size_t i = 0; i < out.size(); ++i) {
          //fprintf(stderr, "%d%s", out[i], (i + 1 < out.size() ? ", " : ""));
        }
       //printf(stderr, "}\n");
    }

    return out;
}


// 3) Robustly detect if a call feeds this BB's terminator (through SSA wrappers)
static bool tree_derives_from(tree t, tree base, int depth = 6) {
    if (!t || !base) return false;
    if (t == base) return true;
    if (TREE_CODE(t) != SSA_NAME || depth <= 0) return false;

    gimple* def = SSA_NAME_DEF_STMT(t);
    if (!def) return false;

    if (is_gimple_call(def)) {
        return gimple_call_lhs(def) == base;
    } else if (is_gimple_assign(def)) {
        if (gimple_num_ops(def) >= 2) {
            tree r1 = gimple_assign_rhs1(def);
            if (r1 == base) return true;
            if (TREE_CODE(r1) == SSA_NAME && tree_derives_from(r1, base, depth - 1)) return true;
        }
        if (gimple_num_ops(def) >= 3) {
            tree r2 = gimple_assign_rhs2(def);
            if (r2 == base) return true;
            if (TREE_CODE(r2) == SSA_NAME && tree_derives_from(r2, base, depth - 1)) return true;
        }
        if (gimple_num_ops(def) >= 4) {
            tree r3 = gimple_assign_rhs3(def);
            if (r3 == base) return true;
            if (TREE_CODE(r3) == SSA_NAME && tree_derives_from(r3, base, depth - 1)) return true;
        }
    }
    return false;
}

// 4) Choose (TRUE,FALSE) destinations deterministically from candidate states
static inline std::pair<int,int> pick_true_false(const std::vector<int>& cand) {
    int t = -1, f = -1;
    for (int s : cand) {
        if (t < 0) t = s;
        else if (f < 0 && s != t) { f = s; break; }
    }
    if (t >= 0 && f < 0) f = t;  // only one candidate → both arms same
    return {t, f};
}


static std::unordered_map<gimple*, int> cond_to_ifelse_id;
std::unordered_map<tree, int> object_state_branch;
std::unordered_map<tree, int> object_stateper_branch;
std::map<int, std::map<tree, int>> branch_state_map;





// Return the controlling terminator (COND/SWITCH) that leads into 'bb'.
// Skips loop back-edges so loops are not confused with branches.
static gimple* controlling_terminator_of(basic_block bb) {
    if (!bb) return nullptr;
    edge e; edge_iterator ei;
    FOR_EACH_EDGE(e, ei, bb->preds) {
        basic_block pred = e->src;
        gimple_stmt_iterator gsi = gsi_last_bb(pred);
        if (gsi_end_p(gsi)) continue;
        gimple* last = gsi_stmt(gsi);
        enum gimple_code code = gimple_code(last);
        if (code == GIMPLE_COND || code == GIMPLE_SWITCH) {
            if (e->flags & EDGE_DFS_BACK) continue; // skip loop back-edges
            return last;
        }
    }
    return nullptr;
}

// Stable id for a given controlling terminator (so both arms share it).
static int get_if_else_id_for_terminator(gimple* term) {
    if (!term) return -1;
    auto it = cond_to_ifelse_id.find(term);
    if (it != cond_to_ifelse_id.end()) return it->second;
    int id = if_else_counter++;
    cond_to_ifelse_id[term] = id;
    return id;
}

// Stable id for a successor BB (uses its controlling predecessor terminator).
int get_if_else_id_for_bb(basic_block bb) {
    if (!bb) return -1;
    if (auto it = bb_to_if_else_id.find(bb); it != bb_to_if_else_id.end())
        return it->second;

    gimple* term = controlling_terminator_of(bb);
    if (!term) return -1;

    int id = get_if_else_id_for_terminator(term);
    bb_to_if_else_id[bb] = id;
    return id;
}

// True if BB is reached from an if/switch (non loop-back).
bool is_inside_if_or_switch(basic_block bb) {
    return controlling_terminator_of(bb) != nullptr;
}








//  Branch context & strict join 

using Bitset = std::bitset<64>;      
int NUM_STATES = -1;        // discovered elsewhere

struct BranchCtx {
    int expected = 0;
    int visited  = 0;
    std::unordered_map<tree,int> pre;
    std::unordered_map<tree,std::vector<Bitset>> outs;

    // TRUE/FALSE seed per object
    std::unordered_map<tree, std::pair<int,int>> arm_choice;

    // final state per arm, per object
    std::unordered_map<tree, std::vector<int>> finals;
};


 
static std::pair<int,int> pick_true_false(const Bitset& bs) {
    int d1 = -1, d2 = -1;
    for (int i = 0; i < (int)bs.size(); ++i) if (bs.test(i)) {
        if (d1 < 0) d1 = i;
        else if (d2 < 0 && i != d1) { d2 = i; break; }
    }
    if (d1 >= 0 && d2 < 0) d2 = d1; // single candidate → both arms same
    return {d1, d2};
}

static std::unordered_map<int, BranchCtx> IFCTX; // ifid -> context

// Normalize object (base + aliases); reuse your helpers
static inline tree norm_obj(tree t) {
    t = get_original_object(t);
    t = track_all_aliases(t);
    return t;
}

static BranchCtx& ensure_ifctx(int id, gimple* term) {
    auto& ctx = IFCTX[id];
    if (ctx.expected == 0 && term) {
        switch (gimple_code(term)) {
            case GIMPLE_COND:   ctx.expected = 2; break;
            case GIMPLE_SWITCH: {
                const gswitch* gs = as_a<const gswitch*>(term);
                int n = gimple_switch_num_labels(gs);
                ctx.expected = std::max(1, n);
                break;
            }
            default:            ctx.expected = 2; break;
        }
    }
    if (ctx.expected == 0) ctx.expected = 2;
    return ctx;
}

static void record_pre_state(int ifid, tree obj, int pre_state) {
    auto& ctx = IFCTX[ifid];
    obj = norm_obj(obj);
    if (!obj) return;
    if (!ctx.pre.count(obj)) ctx.pre[obj] = pre_state;
}

static void record_arm_out_set(int ifid, tree obj, const Bitset& outset) {
    auto& ctx = IFCTX[ifid];
    obj = norm_obj(obj);
    if (!obj) return;
    ctx.outs[obj].push_back(outset);
}

static void end_arm(int ifid) {
    IFCTX[ifid].visited++;
}

// Intersect all arm out-sets per object; require a single concrete state.
static bool try_finalize_if_group(int ifid) {
    auto it = IFCTX.find(ifid);
    if (it == IFCTX.end()) return false;
    auto& ctx = it->second;
    if (ctx.visited < ctx.expected) return false;

    for (auto& kv : ctx.outs) {
        tree obj = kv.first;
        const auto& vec = kv.second;
        if (vec.empty()) continue;

        Bitset inter = vec.front();
        for (size_t i = 1; i < vec.size(); ++i) inter &= vec[i];

        if (inter.count() != 1) {
            error_at(UNKNOWN_LOCATION,
                     "Typestate: branches merge with %zu possible states (need exactly 1).",
                     (size_t)inter.count());
            // fallback: keep pre-state
            auto itp = ctx.pre.find(obj);
            if (itp != ctx.pre.end())
                state_manager.object_states[obj] = itp->second;
            continue;
        }

        int merged = -1;
        for (int s = 0; s < (int)inter.size(); ++s) if (inter.test(s)) { merged = s; break; }
        if (merged >= 0)
            state_manager.object_states[obj] = merged;
    }

    IFCTX.erase(it);
    return true;
}


// helper: does pred_bb → bb correspond to the TRUE edge?
static bool edge_is_true_successor(basic_block pred_bb, basic_block bb) {
    edge e; edge_iterator ei;
    FOR_EACH_EDGE(e, ei, pred_bb->succs) {
        if (e->dest == bb) return (e->flags & EDGE_TRUE_VALUE) != 0;
    }
    return false;
}


enum CondOp { OP_EQ, OP_NE, OP_OTHER };

// Helper: wide-int -> signed host long long using the node's type precision
static inline long long to_ll_with_prec(const tree t) {
    // t is assumed INTEGER_CST by callers that use this; guard anyway.
    if (!t || TREE_CODE(t) != INTEGER_CST) return 0;
    return wi::to_wide(t, TYPE_PRECISION (TREE_TYPE (t))).to_shwi();
}

static bool is_zero(const tree t) {
    if (!t) return false;
    if (TREE_CODE(t) == INTEGER_CST)      return integer_zerop(t);
    if (TREE_CODE(t) == CONST_DECL) {
        tree v = DECL_INITIAL(t);
        return v && TREE_CODE(v) == INTEGER_CST && integer_zerop(v);
    }
    return false;
}

static bool const_name_and_value(tree t, std::string& name_out, long long& val_out) {
    name_out.clear(); val_out = 0;
    if (!t) return false;

    if (TREE_CODE(t) == CONST_DECL) {
        if (DECL_NAME(t)) name_out = IDENTIFIER_POINTER(DECL_NAME(t));
        tree v = DECL_INITIAL(t);
        if (v && TREE_CODE(v) == INTEGER_CST)
            val_out = to_ll_with_prec(v);     // <-- pass precision
        return true;
    }

    if (TREE_CODE(t) == INTEGER_CST) {
        val_out = to_ll_with_prec(t);          // <-- pass precision
        name_out = std::to_string(val_out);
        return true;
    }

    return false;
}

// Extracts: operator (== or !=), the constant 'K', and whether 'res' is on LHS.
// Returns true iff pattern is (res ==/!= K) or (K ==/!= res) with K a const/enum const.
static bool extract_eq_ne_with_const(gimple* term, tree res,
                                     CondOp& op_out, long long& K_out,
                                     bool& res_on_lhs, std::string& K_name)
{
    op_out = OP_OTHER; K_out = 0; res_on_lhs = false; K_name.clear();
    if (!term || gimple_code(term) != GIMPLE_COND || !res || TREE_CODE(res) != SSA_NAME)
        return false;

    enum tree_code op = gimple_cond_code(term);          // EQ_EXPR / NE_EXPR / ...
    if (op != EQ_EXPR && op != NE_EXPR) return false;

    tree L = gimple_cond_lhs(term);
    tree R = gimple_cond_rhs(term);

    auto is_res = [&](tree t){
        return t == res || (t && TREE_CODE(t) == SSA_NAME && ssa_derives_from(t, res));
    };

    // res ? K
    if (is_res(L) && (TREE_CODE(R) == INTEGER_CST || TREE_CODE(R) == CONST_DECL)) {
        long long Kv = 0; std::string Kn;
        const_name_and_value(R, Kn, Kv);
        op_out = (op == EQ_EXPR) ? OP_EQ : OP_NE;
        K_out = Kv; K_name = Kn; res_on_lhs = true;
        return true;
    }

    // K ? res
    if (is_res(R) && (TREE_CODE(L) == INTEGER_CST || TREE_CODE(L) == CONST_DECL)) {
        long long Kv = 0; std::string Kn;
        const_name_and_value(L, Kn, Kv);
        op_out = (op == EQ_EXPR) ? OP_EQ : OP_NE;
        K_out = Kv; K_name = Kn; res_on_lhs = false;
        return true;
    }

    return false;
}


static inline tree norm_obj_key(tree t) {
    t = get_original_object(t);
    return track_all_aliases(t);
}

static void record_final_state(int ifid, tree obj, int final_state) {
    auto& ctx = IFCTX[ifid];
    obj = norm_obj_key(obj);
    if (!obj) return;
    ctx.finals[obj].push_back(final_state);
}


void defer_branch_handling(int branch_id_param,
                           tree obj,
                           const std::string& method_name,
                           location_t location,
                           gimple* site_stmt)
{
    tree obj_key = norm_obj_key(obj);
    if (!obj_key) return;

    // Find current arm BB and its controlling terminator
    basic_block bb_here = site_stmt ? gimple_bb(site_stmt) : nullptr;
    if (!bb_here) {
        for (basic_block b = ENTRY_BLOCK_PTR_FOR_FN(cfun); b; b = b->next_bb) {
            gimple_stmt_iterator gsi = gsi_last_bb(b);
            if (!gsi_end_p(gsi)) { bb_here = b; break; }
        }
    }
    gimple* term = controlling_terminator_of(bb_here);

    // Recompute the canonical branch id from the terminator; use this id everywhere below
    int branch_id = (term ? get_if_else_id_for_terminator(term) : branch_id_param);
    if (term && branch_id >= 0 && branch_id != branch_id_param) {
        fprintf(stderr,
            "Debug: Branch id mismatch: passed=%d, recomputed=%d. Using recomputed id.\n",
            branch_id_param, branch_id);
    }

    // Ensure IFCTX exists for this branch id (sets expected arm count)
    if (branch_id >= 0 && term) {
        (void)ensure_ifctx(branch_id, term);
    }

    //  pre-branch state ONCE per (branch_id, object)
    if (branch_state_map[branch_id].find(obj_key) == branch_state_map[branch_id].end()) {
        if (state_manager.object_states.find(obj_key) == state_manager.object_states.end())
            state_manager.object_states[obj_key] = 0;
        branch_state_map[branch_id][obj_key] = state_manager.object_states[obj_key];
    //    fprintf(stderr,
      //      "Debug: Stored pre-branch state for object %p in branch group %d: %d\n",
         //   (void*)obj_key, branch_id, state_manager.object_states[obj_key]);
    }
    const int pre_branch = branch_state_map[branch_id][obj_key];

 
    if (object_stateper_branch.find(obj_key) == object_stateper_branch.end())
        object_stateper_branch[obj_key] = pre_branch;

    //  Seed once using the stored TRUE/FALSE plan 
    if (object_stateper_branch[obj_key] == pre_branch) {
        auto itCtx = IFCTX.find(branch_id);
        if (itCtx == IFCTX.end()) {
        //  fprintf(stderr, "Debug: No IFCTX for branch %d; cannot seed.\n", branch_id);
        } else if (!term) {
        //  fprintf(stderr, "Debug: No controlling terminator; cannot seed.\n");
        } else {
            auto& ctx = itCtx->second;
            auto itPlan = ctx.arm_choice.find(obj_key);
            
            if (itPlan == ctx.arm_choice.end()) {
               fprintf(stderr,
                    "Debug: Missing arm plan for object %p in branch %d; staying at pre=%d.\n",
                    (void*)obj_key, branch_id, pre_branch);
            } else {
                bool is_true_arm = edge_is_true_successor(gimple_bb(term), bb_here);
                int seed = is_true_arm ? itPlan->second.first : itPlan->second.second;
                if (seed >= 0) {
                    object_stateper_branch[obj_key] = seed;
                  fprintf(stderr,
                     "Debug: Entering %s arm (id=%d) for object %p: apply seed=%d "
                      "(plan TRUE→%d, FALSE→%d), pre=%d.\n",
                  is_true_arm ? "TRUE" : "FALSE",
                      branch_id, (void*)obj_key, seed,
                      itPlan->second.first, itPlan->second.second, pre_branch);
                }
            }
        }
    }


    int cur = object_stateper_branch[obj_key];
  //fprintf(stderr,
   //   "Debug: Entering branch handling for object %p in branch state %d (pre-branch %d) for method '%s'.\n",
    //  (void*)obj_key, cur, pre_branch, method_name.c_str());

    // Compute transitions from the branch-local state
    auto cand = transitions_for(cur, method_name, location);

    if (cand.empty()) {
      //  error_at(location,
        //    "Debug: No transition applies for '%s' in branch (curr=%d).\n",
         //   method_name.c_str(), cur, pre_branch);
       state_manager.method_calls[obj_key].push_back(method_name);
        return;
    }
    if (cand.size() > 1) {
        fprintf(stderr,
            "Debug: Ambiguous transitions for '%s' in branch state %d (", method_name.c_str(), cur);
        for (int s : cand) fprintf(stderr, "%d ", s);
        fprintf(stderr, "); deferring.\n");
        state_manager.method_calls[obj_key].push_back(method_name);
        return;
    }

    int next = cand.front();
  //  fprintf(stderr,
      //  "Debug: Method '%s' changed branch-local state from %d to %d for object %p.\n",
      //  method_name.c_str(), cur, next, (void*)obj_key);
    object_stateper_branch[obj_key] = next;

    state_manager.method_calls[obj_key].push_back(method_name);
}



void finalize_branch_states(tree obj) {
    tree obj_address = get_original_object(obj);

    // Resolve aliases to track the actual object
    tree resolved_obj = track_all_aliases(obj_address);
    if (!resolved_obj) {
        //printf("Debug: Could not resolve object aliases for %p.\n", (void*)obj_address);
        return;
    }

    for (const auto& [objbr, state] : object_state_branch) {
        state_manager.object_states[objbr] = state;
     printf("Debug: Finalized state for object %p to %d.\n", (void*)objbr, state);
    }
    object_state_branch.clear();
state_manager.object_states[resolved_obj] =  object_stateper_branch[resolved_obj]; // Clear branch-specific states after finalization
     int current_state =  object_stateper_branch[resolved_obj];
    // printf("Debug: object %p in state %d.\n", (void*)resolved_obj, current_state);

   
}

bool check_if_else_completion() {
    int if_else_id = current_if_else_id;
    if_else_context_map[if_else_id].second++;

    int total_branches = if_else_context_map[if_else_id].first;
    int completed_branches = if_else_context_map[if_else_id].second;
    bool check = false;
   //printf("Debug: Checking completion for if-else block %d. Total branches = %d, Completed branches = %d.\n",
          // if_else_id, total_branches, completed_branches);

    if (completed_branches == total_branches ) {
        if_else_status[if_else_id] = true;
        finalize_if_else_block(if_else_id);
       
     check =true;
    }
    else {
        // printf("not complete");
       check=false;
    }
    return check;
}


void end_branch_processing(tree obj /*unused but kept for signature compatibility*/) {
    // Find a representative BB (your current approach)
    basic_block bb = nullptr;
    for (basic_block b = ENTRY_BLOCK_PTR_FOR_FN(cfun); b; b = b->next_bb) {
        gimple_stmt_iterator gsi = gsi_last_bb(b);
        if (!gsi_end_p(gsi)) bb = b;
    }
    if (!bb) return;

    gimple* term = controlling_terminator_of(bb);
    if (!term) return;

    int ifid = get_if_else_id_for_terminator(term);
    if (ifid < 0) return;

    // Mark this arm complete (bumps IFCTX[ifid].visited)
    end_arm(ifid);

    
    auto it = IFCTX.find(ifid);
    if (it != IFCTX.end()) {
        auto& ctx = it->second;

        // If all arms have been visited, compare finals for each tracked object
        if (ctx.visited >= ctx.expected) {
            for (auto& kv : ctx.finals) {
                tree obj_key = kv.first;
                const std::vector<int>& finals = kv.second;

                if (!finals.empty()) {
                    bool all_same = true;
                    int first = finals.front();
                    for (int s : finals) {
                        if (s != first) { all_same = false; break; }
                    }
                    if (all_same) {
                        fprintf(stderr,
                            "Debug: Branch %d: all %d arms end in the SAME state %d for object %p.\n",
                            ifid, (int)finals.size(), first, (void*)obj_key);
                    } else {
                        fprintf(stderr,
                            "Debug: Branch %d: arms end in DIFFERENT states for object %p: ",
                            ifid, (void*)obj_key);
                        for (size_t i = 0; i < finals.size(); ++i)
                            fprintf(stderr, "%s%d", (i?",":""), finals[i]);
                        fprintf(stderr, ".\n");
                    }
                }
            }
        }
    }
    //END LOGGING BLOCK

    // Now finalize (this may erase IFCTX[ifid])
    try_finalize_if_group(ifid);

    // keep your legacy counters/flags if other parts rely on them
    // branch_context_map.clear();
    // is_new_branch = true;
    // ++current_branch_id;
}


static bool both_arms_end_same_state(int ifid, tree obj, int* out_state = nullptr) {
    auto it = IFCTX.find(ifid);
    if (it == IFCTX.end()) return false;
    auto key = norm_obj_key(obj);
    auto itv = it->second.finals.find(key);
    if (itv == it->second.finals.end()) return false;

    const auto& v = itv->second;
    if ((int)v.size() < it->second.expected) return false; // not all arms processed yet

    int first = v.front();
    for (int s : v) if (s != first) {
        if (out_state) *out_state = -1;
        return false;
    }
    if (out_state) *out_state = first;
    return true;
}






// hepler Function to get readble C++ names
std::string demangle(const char* mangled) {
    if (!mangled || !*mangled) {
        return "";
    }
    int status = 0;
    char* demangled_name = abi::__cxa_demangle(mangled, nullptr, nullptr, &status);
    std::string result;
    if (status == 0 && demangled_name) {
        result = demangled_name;
        free(demangled_name);
    } else {
        result = mangled;
    }
    return result;
}


static std::string base_type_org;


// Function to detect if a tree node represents an object subject to typestate checking or a subtype of
// such an object. This ensures that in cases of inheritance, both supertypes and their subtypes
// are checked against typestate rules.

bool is_object_of_Flagged(tree arg, tree& obj_address) {
    obj_address = nullptr;

    if (!arg) {
       
        return false;
    }

   
    tree resolved_arg = track_all_aliases(get_original_object(arg));
    if (!resolved_arg) {
       
        return false;
    }

    tree type = TREE_TYPE(resolved_arg);
    if (!type) {
       
        return false;
    }

   
    if (TREE_CODE(type) != RECORD_TYPE) {
     
        return false;
    }

    const char* type_name = nullptr;
    if (TYPE_NAME(type) && DECL_NAME(TYPE_NAME(type))) {
        type_name = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(type)));
    }

    if (!type_name) {
       
        return false;
    }


    if (TypestateClassConnector_args.empty() || TypestateClassConnector_args[0].empty()) {
       
        return false;
    }

    if (strcmp(type_name, TypestateClassConnector_args[0].c_str()) == 0) {
        obj_address = resolved_arg;
       
        return true;
    }

    // Check base classes for type match
    tree binfo = TYPE_BINFO(type);
    if (!binfo) {
       
        return false;
    }

    int num_bases = BINFO_N_BASE_BINFOS(binfo);
    if (num_bases < 0 || num_bases > 100) {
   
        return false;
    }

    for (int i = 0; i < num_bases; ++i) {
        tree base_binfo = BINFO_BASE_BINFO(binfo, i);
        if (!base_binfo) {
            continue;
        }

        tree base_type = BINFO_TYPE(base_binfo);
        if (!base_type) {

            continue;
        }

        const char* base_type_name = nullptr;
        if (TYPE_NAME(base_type) && DECL_NAME(TYPE_NAME(base_type))) {
            base_type_name = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(base_type)));
        }

        if (!base_type_name) {
            continue;
        }
       

        if (strcmp(base_type_name, TypestateClassConnector_args[0].c_str()) == 0) {
            obj_address = resolved_arg;
   
            return true;
        }
    }

    return false;
}




/**
 * This is part of an inter-procedural analysis.
 * It starts by analysing GIMPLE statements from the main function. Whenever there is a function call,
 * the analysis expands to process and analyse statements within the called functions.
 * Whenever there is a call to a state transition function for a flagged object,
 * it is validated against the typestate rules.
 */

#include <bitset>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>


//  PSA guard: true while the path-sensitive driver is running 
bool g_psa_enabled = false;

//  Strict-join policy toggle (set true to enforce identical singleton states) 
static bool g_strict_join = true;





extern tree get_original_object(tree);
extern tree track_all_aliases(tree);



// Discover number of states from  Typestate_Rules
inline int discover_num_states() {
    int mx = -1;
    for (auto &kv : Typestate_Rules) {
        mx = std::max(mx, kv.first);
        for (auto &p : kv.second) mx = std::max(mx, p.second);
    }
    return mx + 1;
}


struct ObjectId {
    tree t;
    friend bool operator==(const ObjectId& a, const ObjectId& b) { return a.t == b.t; }
};
struct ObjectIdHash {
    size_t operator()(const ObjectId& x) const { return reinterpret_cast<size_t>(x.t); }
};

// Normalize an object root once (base + alias resolution)
inline ObjectId norm_object(tree obj) {
    tree base = get_original_object(obj);
    tree ali  = track_all_aliases(base);
    return ObjectId{ ali ? ali : base };
}

// Facts at a program point: object -> set of possible states
struct Facts {
    std::unordered_map<ObjectId, Bitset, ObjectIdHash> m;
    bool operator==(const Facts& r) const { return m == r.m; }
    bool operator!=(const Facts& r) const { return !(*this == r); }
};

// For the tracked object, remember what the *first* predecessor proposes
// at a given successor block; later predecessors must match it exactly.
static std::unordered_map<basic_block, Bitset>     JOIN_EXPECTED_FOR_BB;
static std::unordered_map<basic_block, location_t> JOIN_ERROR_LOC;

// Meet at joins: intersection (state must be possible on ALL incoming paths)
inline Facts meet(const Facts& A, const Facts& B) {
    Facts R;
    std::unordered_set<ObjectId, ObjectIdHash> keys;
    for (auto &kv : A.m) keys.insert(kv.first);
    for (auto &kv : B.m) keys.insert(kv.first);
    for (auto &o : keys) {
        Bitset a{}, b{};
        if (auto ia = A.m.find(o); ia != A.m.end()) a = ia->second;
        if (auto ib = B.m.find(o); ib != B.m.end()) b = ib->second;
        R.m[o] = (a & b); // (could be empty)
    }
    return R;
}

// DFA helpers using your Typestate_Rules
inline Bitset step_states_union(const Bitset& cur, const std::string& method) {
    Bitset next;
    for (int s = 0; s < NUM_STATES; ++s) if (cur.test(s)) {
        auto it = Typestate_Rules.find(s);
        if (it == Typestate_Rules.end()) continue;
        for (auto &e : it->second) if (e.first == method) {
            int dst = e.second;
            if (0 <= dst && dst < NUM_STATES) next.set(dst);
        }
    }
    return next;
}
inline bool call_valid_from_all(const Bitset& cur, const std::string& method) {
    for (int s = 0; s < NUM_STATES; ++s) if (cur.test(s)) {
        bool ok = false;
        auto it = Typestate_Rules.find(s);
        if (it != Typestate_Rules.end()) {
            for (auto &e : it->second) if (e.first == method) { ok = true; break; }
        }
        if (!ok) return false;
    }
    return true;
}
inline std::vector<int> all_dsts_for(int src, const std::string& method) {
    std::vector<int> v;
    auto it = Typestate_Rules.find(src);
    if (it == Typestate_Rules.end()) return v;
    for (auto &e : it->second) if (e.first == method) v.push_back(e.second);
    return v;
}
inline bool method_is_guarded(const Bitset& cur, const std::string& method) {
    for (int s = 0; s < NUM_STATES; ++s) if (cur.test(s)) {
        if (all_dsts_for(s, method).size() > 1) return true;
    }
    return false;
}
inline Bitset get_obj_states_from_facts(const Facts& F, tree obj_address) {
    auto it = F.m.find(norm_object(obj_address));
    return (it == F.m.end()) ? Bitset{} : it->second;
}



static std::unordered_map<basic_block, Facts> IN_facts;     // facts at BB entry
static std::unordered_map<basic_block, Facts> CUR_facts;    // running facts while scanning BB
static std::unordered_map<basic_block, Facts> EDGE_TRUE_OUT;  // edge-specific OUT (true)
static std::unordered_map<basic_block, Facts> EDGE_FALSE_OUT; // edge-specific OUT (false)

inline Facts& ensure_bb_facts(gimple* stmt) {
    basic_block bb = gimple_bb(stmt);
    auto it = CUR_facts.find(bb);
    if (it == CUR_facts.end()) CUR_facts[bb] = IN_facts[bb]; // seed with IN at BB entry
    return CUR_facts[bb];
}
inline Bitset get_states(Facts& F, tree obj) { return F.m[norm_object(obj)]; }
inline void set_states(Facts& F, tree obj, const Bitset& bs) { F.m[norm_object(obj)] = bs; }

// Does a call feed this GIMPLE_COND?
static bool call_feeds_terminator_cond(gimple* call_stmt, gimple* term) {
    if (!call_stmt || !term) return false;
    if (gimple_code(term) != GIMPLE_COND) return false;

    tree res = gimple_call_lhs(call_stmt);
    if (!res || TREE_CODE(res) != SSA_NAME) return false;

    tree lhs = gimple_cond_lhs(term);
    tree rhs = gimple_cond_rhs(term);
    if (lhs == res || rhs == res) return true;
    return tree_derives_from(lhs, res) || tree_derives_from(rhs, res);
}


void process_statement_in_function(gimple* stmt, tree obj_address) {
    if (!stmt) return;

    if (is_gimple_call(stmt)) {
        tree called_fn = gimple_call_fndecl(stmt);
        if (called_fn) {
            const char* callee_name_raw = IDENTIFIER_POINTER(DECL_NAME(called_fn));
            std::string callee_name = callee_name_raw ? demangle(callee_name_raw) : "unknown_function";

            tree this_arg = NULL_TREE;
            if (DECL_CONTEXT(called_fn) && TREE_CODE(DECL_CONTEXT(called_fn)) == RECORD_TYPE &&
                !DECL_STATIC_FUNCTION_P(called_fn)) {
                this_arg = gimple_call_arg(stmt, 0);
            }

            if (this_arg) {
                this_arg = get_original_object(this_arg);
                tree resolved_this_arg = track_all_aliases(this_arg);

                auto handle_on_obj = [&](tree which_obj) {
                    // path-sensitive check/step (replaces Typestate_Checking) 
                    Facts& facts = ensure_bb_facts(stmt);
                    Bitset cur = get_states(facts, which_obj);

                    // 1) validity check at the call site
                    if (!call_valid_from_all(cur, callee_name)) {
                       error_at(gimple_location(stmt),
                             "Typestate: call '%s' not valid from all possible states here.",
                                callee_name.c_str());
                    }

                    // 2) split if guarded call feeds this BB's terminator condition
                    gimple_stmt_iterator tail = gsi_last_bb(gimple_bb(stmt));
                    gimple* term = gsi_end_p(tail) ? nullptr : gsi_stmt(tail);

                    if (call_feeds_terminator_cond(stmt, term) && method_is_guarded(cur, callee_name)) {
                        Bitset union_next = step_states_union(cur, callee_name);

                        // choose two destinations deterministically
                        int d1 = -1, d2 = -1;
                        for (int s = 0; s < NUM_STATES; ++s) if (cur.test(s)) {
                            auto v = all_dsts_for(s, callee_name);
                            for (int d : v) {
                                if (d1 < 0) d1 = d;
                                else if (d2 < 0 && d != d1) { d2 = d; break; }
                            }
                            if (d2 >= 0) break;
                        }
                        if (d1 < 0) d1 = 0;
                        if (d2 < 0) d2 = d1;

                        Bitset next_true{}, next_false{};
                        for (int d = 0; d < NUM_STATES; ++d) if (union_next.test(d)) {
                            if (d == d1)  next_true.set(d);
                            if (d == d2)  next_false.set(d);
                            if (d != d1 && d != d2) { next_true.set(d); next_false.set(d); } // conservative
                        }

                        basic_block bb = gimple_bb(stmt);
                        EDGE_TRUE_OUT[bb]  = facts; set_states(EDGE_TRUE_OUT[bb],  which_obj, next_true);
                        EDGE_FALSE_OUT[bb] = facts; set_states(EDGE_FALSE_OUT[bb], which_obj, next_false);
                        // do NOT also write back to 'facts' here
                    } else {
                        Bitset next = step_states_union(cur, callee_name);
                        set_states(facts, which_obj, next);
                    }
                };

                if (resolved_this_arg == obj_address) {
                    handle_on_obj(obj_address);
                } else {
                    tree final_alias = track_all_aliases(resolved_this_arg);
                    if (final_alias == obj_address) {
                        handle_on_obj(obj_address);
                    } else {
                        // not the tracked object → ignore
                    }
                }

                if (callee_name == "operator->") {
                    tree lhs = gimple_call_lhs(stmt);
                    if (lhs) {
                        // keep your alias book-keeping
                        state_manager.register_alias(lhs, obj_address);
                    }
                }
            }
        }
    }
    else if (is_gimple_assign(stmt)) {
       
        tree lhs = gimple_assign_lhs(stmt);
        tree rhs = gimple_assign_rhs1(stmt);

        if (TREE_CODE(lhs) == COMPONENT_REF) {
            tree base = TREE_OPERAND(lhs, 0);
            tree member = TREE_OPERAND(lhs, 1);

            base = get_original_object(base);
            tree resolved_base = track_all_aliases(base);

            tree this_param = DECL_ARGUMENTS(current_function_decl);
            if (resolved_base == this_param) {
                tree resolved_rhs = track_all_aliases(get_original_object(rhs));
                if (resolved_rhs == obj_address) {
                    state_manager.register_alias(lhs, obj_address);
                }
            }
        }
    }
}

// Seed facts for entry (set your real initial state index here)
static Facts initial_facts_for_object(tree obj_address) {
    Facts F;
    Bitset start{}; start.set(0); // assume state 0 = Idle (change if needed)
    F.m[norm_object(obj_address)] = start;
    return F;
}




function* g_psa_fn            = nullptr;   // which function PSA is analyzing
tree      g_psa_tracked_obj   = nullptr;   // root object PSA is analyzing



// Normalize to a stable root (matches what your maps use)
inline tree root_of(tree t) {
    return track_all_aliases(get_original_object(t));
}

// RAII guard to set the PSA context for this run
struct PsaGuard {
    function* saved_fn;
    tree      saved_obj;
    bool      saved_enabled;
    PsaGuard(function* fn, tree obj) {
        saved_fn      = g_psa_fn;
        saved_obj     = g_psa_tracked_obj;
        saved_enabled = g_psa_enabled;

        g_psa_fn          = fn;
        g_psa_tracked_obj = root_of(obj);
        g_psa_enabled     = true;
    }
    ~PsaGuard() {
        g_psa_fn          = nullptr;
        g_psa_tracked_obj = nullptr;
        g_psa_enabled     = false;
    }
};


inline void ts_check_guarded(tree obj, const std::string& name, location_t loc) {
  if (!g_psa_enabled) state_manager.Typestate_Checking(obj, name, loc);
}

void analyze_function_typestate(function* fn, tree obj_address) {
     PsaGuard _guard_(fn, obj_address);
    if (NUM_STATES < 0) NUM_STATES = discover_num_states();

   
    IN_facts.clear();
    CUR_facts.clear();
    EDGE_TRUE_OUT.clear();
    EDGE_FALSE_OUT.clear();
    JOIN_EXPECTED_FOR_BB.clear();
    JOIN_ERROR_LOC.clear();

    // Init empty IN facts for all blocks (includes fake ENTRY/EXIT)
    for (basic_block b = ENTRY_BLOCK_PTR_FOR_FN(fn); b; b = b->next_bb)
        IN_facts[b] = Facts{};

    // Seed the FIRST REAL block (ENTRY is fake in GCC 13)
    basic_block entry_fb = ENTRY_BLOCK_PTR_FOR_FN(fn);
    edge e0 = nullptr; edge_iterator ei0;
    FOR_EACH_EDGE (e0, ei0, entry_fb->succs) { break; }
    if (!e0) return;                   // malformed CFG
    basic_block start_bb = e0->dest;

    // Initial state for the tracked object (adjust index if Idle != 0)
    {
        Facts F;
        Bitset start{}; start.set(0);  // state 0 = Idle (or your real start)
        F.m[norm_object(obj_address)] = start;
        IN_facts[start_bb] = F;
    }

    std::queue<basic_block> wl;
    wl.push(start_bb);

    while (!wl.empty()) {
        basic_block bb = wl.front(); wl.pop();

        // Start from current IN facts for this bb
        CUR_facts[bb] = IN_facts[bb];

        // Clear per-bb edge outs
        EDGE_TRUE_OUT.erase(bb);
        EDGE_FALSE_OUT.erase(bb);

        //  Walk statements in BB (updates CUR_facts[bb]) 
        for (gimple_stmt_iterator gsi = gsi_start_bb(bb); !gsi_end_p(gsi); gsi_next(&gsi)) {
            gimple* s = gsi_stmt(gsi);
            process_statement_in_function(s, obj_address);
        }

        // Default OUT for edges without split
        Facts out = CUR_facts[bb];

        // Clear expectations for these successors on this visit to avoid stale compares
        {
            edge e1; edge_iterator ei1;
            FOR_EACH_EDGE(e1, ei1, bb->succs) {
                JOIN_EXPECTED_FOR_BB.erase(e1->dest);
                JOIN_ERROR_LOC.erase(e1->dest);
            }
        }

        //  Propagate to successors with MEET; enforce strict join if requested 
        edge e; edge_iterator ei;
        FOR_EACH_EDGE (e, ei, bb->succs) {
            Facts cand = out;
            if ((e->flags & EDGE_TRUE_VALUE)  && EDGE_TRUE_OUT.count(bb))  cand = EDGE_TRUE_OUT[bb];
            if ((e->flags & EDGE_FALSE_VALUE) && EDGE_FALSE_OUT.count(bb)) cand = EDGE_FALSE_OUT[bb];

            if (g_strict_join) {
                // Candidate state set for the tracked object on this incoming edge
                Bitset cand_set = get_obj_states_from_facts(cand, obj_address);

                // Record a reasonable location for join diagnostics (first stmt of successor)
                if (!JOIN_ERROR_LOC.count(e->dest)) {
                    gimple_stmt_iterator it = gsi_start_bb(e->dest);
                    JOIN_ERROR_LOC[e->dest] = gsi_end_p(it) ? UNKNOWN_LOCATION : gimple_location(gsi_stmt(it));
                }

                auto itExp = JOIN_EXPECTED_FOR_BB.find(e->dest);
                if (itExp == JOIN_EXPECTED_FOR_BB.end()) {
                    // First incoming defines the expected set at this join
                    JOIN_EXPECTED_FOR_BB[e->dest] = cand_set;
                } else {
                    Bitset expected = itExp->second;

                    // 1) Both branches must propose EXACTLY the same set
                    if (expected != cand_set) {
                        error_at(JOIN_ERROR_LOC[e->dest],
                                 "Typestate: branches merge with different states for the object "
                                 "(%zu vs %zu possibilities).",
                                 (size_t)expected.count(), (size_t)cand_set.count());
                    }
                    // 2) That set must be a SINGLE concrete state
                    if (cand_set.count() != 1) {
                        error_at(JOIN_ERROR_LOC[e->dest],
                                 "Typestate: state is indeterminate at join "
                                 "(expected a single state, got %zu).",
                                 (size_t)cand_set.count());
                    }
                }
            }

            // Normal meet-based propagation (sound even with strict join enabled)
            Facts joined = IN_facts.count(e->dest) ? meet(IN_facts[e->dest], cand) : cand;
            if (joined != IN_facts[e->dest]) {
                IN_facts[e->dest] = joined;
                wl.push(e->dest); // iterate to fixed point (handles loops)
            }
        }
    }
}



void process_function_body(tree function_decl, tree obj_address) {
    if (!function_decl) return;

    struct function* fn = DECL_STRUCT_FUNCTION(function_decl);
    if (!fn || !fn->cfg) return;

    static std::unordered_set<function*> processed_functions;
    if (processed_functions.find(fn) != processed_functions.end()) return;
    processed_functions.insert(fn);

    push_cfun(fn);
    call_stack.emplace_back();
g_psa_enabled = true;
analyze_function_typestate(fn, obj_address);  // your PSA worklist driver
g_psa_enabled = false;
    // >>> This single call runs the path-sensitive analysis for the one object you passed.
 //   analyze_function_typestate(fn, obj_address);

    pop_cfun();
    call_stack.pop_back();
}



void Analyse_gimple_statement(gimple* stmt,gimple_stmt_iterator* gsi);




static void analyze_switch_cases(basic_block switch_bb, gimple *switch_stmt) {
    if (!switch_stmt || gimple_code(switch_stmt) != GIMPLE_SWITCH)
        return;

    // Cast to gswitch*
    const gswitch *gs = as_a<const gswitch *>(switch_stmt);

    printf("\n[INFO] Found a switch statement in BB %d\n", switch_bb->index);

    // Get the number of cases (including default)
    int num_labels = gimple_switch_num_labels(gs);
    printf("[INFO] Number of cases (including default if present): %d\n", num_labels);

    edge e;
    edge_iterator ei;

    FOR_EACH_EDGE(e, ei, switch_bb->succs) {
        basic_block case_bb = e->dest;

        // Iterate over the labels in the switch statement
        for (int i = 0; i < num_labels; i++) {
            tree label_expr = gimple_switch_label(gs, i);

            // If the edge destination matches, print information
            if (e->dest == case_bb) {
                if (TREE_CODE(label_expr) == CASE_LABEL_EXPR) {
                    tree case_low = CASE_LOW(label_expr);
                    tree case_high = CASE_HIGH(label_expr);

                    if (case_high && case_high != case_low) {
                        // Case range (e.g., case 3 ... 5)
                        printf("[INFO] Case range %ld ... %ld -> BB %d\n",
                               TREE_INT_CST_LOW(case_low),
                               TREE_INT_CST_LOW(case_high),
                               case_bb->index);
                    } else {
                        // Single case value (e.g., case 3)
                        printf("[INFO] Case %ld -> BB %d\n",
                               TREE_INT_CST_LOW(case_low),
                               case_bb->index);
                    }
                } else if (TREE_CODE(label_expr) == LABEL_EXPR) {
                    // Default case
                    printf("[INFO] Default case -> BB %d\n", case_bb->index);
                }
            }
        }
    }
}

// Main function to iterate through basic blocks and detect switches
void Analyse_gimple_in_function(std::unordered_map<tree, tree> param_arg_map = {}) {

    //  detect direct recursion for this function ----------
    function* fn = cfun;
    if (fn && fn->decl && !g_recursive_fn_decls.count(fn->decl)) {
        bool is_rec = false;
        tree self = fn->decl;

        basic_block bb;
        FOR_EACH_BB_FN(bb, fn) {
            for (gimple_stmt_iterator gsi = gsi_start_bb(bb);
                 !gsi_end_p(gsi);
                 gsi_next(&gsi))
            {
                gimple* s = gsi_stmt(gsi);
                if (!is_gimple_call(s)) continue;

                tree callee = gimple_call_fndecl(s);
                if (!callee) continue;

                if (TREE_CODE(callee) == FUNCTION_DECL && callee == self) {
                    is_rec = true;   // found self-call: e.g. AdjustActuators(n-1)
                    break;
                }
            }
            if (is_rec) break;
        }

        if (is_rec) {
            g_recursive_fn_decls.insert(self);
            printf("[INFO] Detected direct recursion in function: %s\n",
                   IDENTIFIER_POINTER(DECL_NAME(self)));
        }
    }
    // ---------- END NEW ----------

    call_stack.emplace_back();
    CallContext& context = call_stack.back();

    tree function_decl = current_function_decl;
    if (function_decl) {
        tree param;
        for (param = DECL_ARGUMENTS(function_decl); param; param = TREE_CHAIN(param)) {
            if (param_arg_map.find(param) != param_arg_map.end()) {
                tree arg = param_arg_map[param];
                arg = get_original_object(arg);
                arg = track_all_aliases(arg);  

                context.alias_map[param] = arg;
                global_alias_map[param] = arg;  
            }
        }
    }

    // Analyse the function body (preserve existing logic)
    basic_block bb;
    FOR_EACH_BB_FN(bb, cfun) {
        gimple_stmt_iterator gsi;
        for (gsi = gsi_start_bb(bb); !gsi_end_p(gsi); gsi_next(&gsi)) {
            gimple* stmt = gsi_stmt(gsi);
            Analyse_gimple_statement(stmt, &gsi);
        }
    }

    // Pop the call context
    call_stack.pop_back();
}



/*───────────────────────────────────────────────────────────────────*\
 |  Examine one call statement                                      
 |  – typestate check on the receiver                                
 |  – propagate operator-> aliases                                    
 |  – build parameter→argument map                                    
 |  – queue callee when <fn,obj,state> not seen before                
\*───────────────────────────────────────────────────────────────────*/


extern bool g_psa_enabled;
extern function* g_psa_fn;
extern tree g_psa_tracked_obj;



std::unordered_map<int, int> branchExpectedCount; // Expected number of branches for each if-else group.
std::unordered_map<int, int> branchVisitedCount;
bool is_if_else_structure_complete(int if_else_id) {
    // If we haven't set an expected count yet, we can't say it's complete.
    if (branchExpectedCount.find(if_else_id) == branchExpectedCount.end() ||
        branchVisitedCount.find(if_else_id) == branchVisitedCount.end())
    {
        return false;
    }
   
    // The structure is complete when the number of visited branches equals the expected count.
    return branchVisitedCount[if_else_id] == branchExpectedCount[if_else_id];
}


std::vector<int> get_candidate_next_states(int current_state, const std::string &method_name) {
    std::vector<int> candidates;
    if (Typestate_Rules.find(current_state) != Typestate_Rules.end()) {
        for (const auto &trans : Typestate_Rules[current_state]) {
            if (trans.first == method_name) {
                candidates.push_back(trans.second);
            }
        }
    }
    return candidates;
}



//std::unordered_map<int, std::unordered_map<tree, int>> branch_state_map;
std::map<tree, std::set<int>> nondet_object_states;
#include "wide-int.h"
#include "tree-cfg.h"


static tree strip_typedefs_quals(tree t) {
    if (!t) return t;
#ifdef TYPE_CANONICAL
    if (TYPE_CANONICAL(t)) return TYPE_CANONICAL(t);
#endif
    return TYPE_MAIN_VARIANT(t) ? TYPE_MAIN_VARIANT(t) : t;
}

static bool method_returns_boolean_or_enum(tree callee_decl) {
    if (!callee_decl || TREE_CODE(callee_decl) != FUNCTION_DECL) return false;
    tree fntype = TREE_TYPE(callee_decl);
    if (!fntype || TREE_CODE(fntype) != FUNCTION_TYPE) return false;
    tree ret = TREE_TYPE(fntype);
    if (!ret) return false;
    ret = strip_typedefs_quals(ret);
    if (TREE_CODE(ret) == BOOLEAN_TYPE)   return true;
    if (TREE_CODE(ret) == ENUMERAL_TYPE)  return true;
    if (TREE_CODE(ret) == INTEGER_TYPE)   return TYPE_PRECISION(ret) == 1; // bool-like typedef
    return false;
}

//  const handling + condition analysis (prints enum name/value) 
static bool const_to_wide_int(tree t, wide_int* out) {
    if (!t || !out) return false;

    if (TREE_CODE(t) == INTEGER_CST) {
        *out = TO_WIDE(t);
        return true;
    }

    if (TREE_CODE(t) == CONST_DECL) {
        tree v = DECL_INITIAL(t);
        if (v) {
            STRIP_NOPS(v); // handle enum/const wrapped in NOP/CONVERT
            if (TREE_CODE(v) == INTEGER_CST) {
                *out = TO_WIDE(v);
                return true;
            }
        }
    }

    return false;
}


#include "tree.h"

// Use once in a common header/source:
#ifndef TO_WIDE
#  if defined(GCCPLUGIN_VERSION) && (GCCPLUGIN_VERSION < 8000)
// GCC 7.x needs precision
#    define TO_WIDE(T_) wi::to_wide((T_), TYPE_PRECISION (TREE_TYPE (T_)))
#  else
// GCC 8+ has the 1-arg overload
#    define TO_WIDE(T_) wi::to_wide((T_))
#  endif
#endif

static bool const_name_and_value(tree t, std::string& name_out, long long* val_out) {
    name_out.clear();
    if (!t) return false;

    if (TREE_CODE(t) == CONST_DECL) {
        if (DECL_NAME(t)) name_out = IDENTIFIER_POINTER(DECL_NAME(t));
        tree v = DECL_INITIAL(t);
        if (v) {
            STRIP_NOPS(v); // handle enum constants or folded expressions wrapped in NOP/CONVERT
            if (TREE_CODE(v) == INTEGER_CST) {
                if (val_out) *val_out = TO_WIDE(v).to_shwi();
                return true;
            }
        }
        return false; // CONST_DECL without INTEGER_CST initializer
    }

    if (TREE_CODE(t) == INTEGER_CST) {
        long long v = TO_WIDE(t).to_shwi();
        if (val_out) *val_out = v;
        name_out = std::to_string(v);
        return true;
    }

    return false;
}


// Does COND compare call result 'res' to a constant K?
static bool cond_compares_res_to_const(gimple* term, tree res, bool* out_is_eq, wide_int* out_val) {
    if (!term || gimple_code(term) != GIMPLE_COND || !res) return false;
    enum tree_code code = gimple_cond_code(term);
    if (code != EQ_EXPR && code != NE_EXPR) return false;

    tree L = gimple_cond_lhs(term);
    tree R = gimple_cond_rhs(term);
    auto is_res = [&](tree t){ return t==res || (TREE_CODE(t)==SSA_NAME && ssa_derives_from(t,res)); };

    if (is_res(L) && const_to_wide_int(R, out_val)) { *out_is_eq = (code==EQ_EXPR); return true; }
    if (is_res(R) && const_to_wide_int(L, out_val)) { *out_is_eq = (code==EQ_EXPR); return true; }
    return false;
}

// Is COND effectively "if (res)" or "if (!res)"?
static bool cond_is_simple_truth_test(gimple* term, tree res, bool* true_when_nonzero) {
    if (!term || gimple_code(term) != GIMPLE_COND || !res) return false;
    enum tree_code code = gimple_cond_code(term);
    tree L = gimple_cond_lhs(term);
    tree R = gimple_cond_rhs(term);
    auto is_res = [&](tree t){ return t==res || (TREE_CODE(t)==SSA_NAME && ssa_derives_from(t,res)); };

    if (code == NE_EXPR || code == EQ_EXPR) {
        if (is_res(L) && is_falsey_const(R)) { *true_when_nonzero = (code==NE_EXPR); return true; }
        if (is_res(R) && is_falsey_const(L)) { *true_when_nonzero = (code==NE_EXPR); return true; }
    }
    return false;
}



struct BranchSemantics {
    bool compares_to_const = false;
    bool is_eq = true;
    wide_int const_val;

    bool simple_truth_test = false;
    bool true_when_nonzero = true;
};

//  prints what it understands about the branch w.r.t the call result
static BranchSemantics analyze_branch_semantics(gimple* term, gimple* call_stmt) {
    BranchSemantics bs{};
    if (!term || gimple_code(term) != GIMPLE_COND || !call_stmt) return bs;

    tree res = gimple_call_lhs(call_stmt);
    if (!res || TREE_CODE(res) != SSA_NAME) return bs;

    if (cond_compares_res_to_const(term, res, &bs.is_eq, &bs.const_val)) {
        bs.compares_to_const = true;

        // Print the exact constant name/value seen in the condition
        tree L = gimple_cond_lhs(term);
        tree R = gimple_cond_rhs(term);
        std::string cname; long long cval = 0;
        if (TREE_CODE(L) == CONST_DECL || TREE_CODE(L) == INTEGER_CST) const_name_and_value(L, cname, &cval);
        else if (TREE_CODE(R) == CONST_DECL || TREE_CODE(R) == INTEGER_CST) const_name_and_value(R, cname, &cval);
    //    fprintf(stderr, "Debug: condition compares call result to const %s (=%lld); TRUE means %s.\n",
          //      cname.c_str(), cval, bs.is_eq ? "== const" : "!= const");
        return bs;
    }

    if (cond_is_simple_truth_test(term, res, &bs.true_when_nonzero)) {
 //       fprintf(stderr, "Debug: condition is simple truth test; TRUE means (res %s 0).\n",
       //         bs.true_when_nonzero ? "!=" : "==");
        bs.simple_truth_test = true;
        return bs;
    }

  //  fprintf(stderr, "Debug: condition analysis inconclusive; using heuristic.\n");
    return bs;
}





static const char* arm_label_then_else(gimple* term, basic_block succ) {
    if (!term) return "?";
    basic_block pred = gimple_bb(term);
    basic_block btrue=nullptr, bfalse=nullptr;
    edge e; edge_iterator ei;
    FOR_EACH_EDGE(e, ei, pred->succs) {
        if (e->flags & EDGE_TRUE_VALUE)  btrue  = e->dest;
        if (e->flags & EDGE_FALSE_VALUE) bfalse = e->dest;
    }
    if (succ == btrue)  return "THEN";
    if (succ == bfalse) return "ELSE";
    return "?";
}


static bool is_end_of_arm_bb(basic_block bb) {
    if (!bb) return false;

    // Must be inside an if/switch arm
    gimple* term = controlling_terminator_of(bb);
    if (!term) return false;
    int ifid = get_if_else_id_for_terminator(term);

    // Heuristic: if this BB has a single successor whose own if-id differs,
    // or the successor has >=2 predecessors (join),  consider this the end.
    edge e; edge_iterator ei;
    int succs = 0;
    basic_block succ_only = nullptr;
    FOR_EACH_EDGE(e, ei, bb->succs) {
        ++succs;
        if (succs == 1) succ_only = e->dest;
    }
    if (succs != 1) return false; // still branching, not a straight fallthrough

    // Successor’s if-id (if any)
    int succ_ifid = get_if_else_id_for_bb(succ_only);

    // If successor has multiple predecessors, it’s likely the join
    int pred_count = 0;
    FOR_EACH_EDGE(e, ei, succ_only->preds) { ++pred_count; }

    // End-of-arm if we’re leaving this if-group OR we’re falling into a join
    if (succ_ifid != ifid) return true;
    if (pred_count >= 2)   return true;

    return false;
}


// Map: function declaration -> recursion bound (n)
static std::unordered_map<tree, int> g_recursion_bounds;
// Any FUNCTION_DECL in here is known to be directly recursive.
static std::unordered_set<tree> g_recursive_fn_names;

static bool is_direct_recursive_call(tree callee_decl) {
    if (!callee_decl || !cfun) return false;
    if (TREE_CODE(callee_decl) != FUNCTION_DECL) return false;
    return (cfun->decl == callee_decl);
}


static void register_recursion_bound(tree fn_decl, int bound) {
    if (!fn_decl) return;
    g_recursion_bounds[fn_decl] = bound;
}

static bool get_recursion_bound(tree fn_decl, int& out_bound) {
    auto it = g_recursion_bounds.find(fn_decl);
    if (it == g_recursion_bounds.end()) return false;
    out_bound = it->second;
    return true;
}

static bool is_function_marked_recursive(tree fn_decl) {
    int dummy;
    return get_recursion_bound(fn_decl, dummy);
}


static bool is_constant_int_bound(tree t, long &out_val) {
    if (!t) return false;

    if (TREE_CODE(t) == INTEGER_CST) {
        out_val = (long)TREE_INT_CST_LOW(t);
        return true;
    }

    if (TREE_CODE(t) == SSA_NAME) {
        gimple *def = SSA_NAME_DEF_STMT(t);
        if (!def) return false;
        if (is_gimple_assign(def) && gimple_assign_single_p(def)) {
            tree rhs = gimple_assign_rhs1(def);
            if (rhs && TREE_CODE(rhs) == INTEGER_CST) {
                out_val = (long)TREE_INT_CST_LOW(rhs);
                return true;
            }
        }
    }

    return false;
}


static bool function_has_direct_recursion(function* fn) {
    if (!fn || !fn->decl) return false;

    tree self = fn->decl;

    basic_block bb;
    FOR_EACH_BB_FN(bb, fn) {
        for (gimple_stmt_iterator gsi = gsi_start_bb(bb);
             !gsi_end_p(gsi);
             gsi_next(&gsi))
        {
            gimple* stmt = gsi_stmt(gsi);
            if (!is_gimple_call(stmt)) continue;

            tree callee = gimple_call_fndecl(stmt);
            if (!callee) continue;

            if (TREE_CODE(callee) == FUNCTION_DECL && callee == self) {
                // Found a direct self-call
                return true;
            }
        }
    }
    return false;
}

static bool is_recursive_transition_method(const std::string& name) {
    for (const auto& rt : g_recursive_transitions) {
        if (rt.method == name) {
            return true;
        }
    }
    return false;
}


static std::unordered_set<std::string> g_recursive_fn_names_str;
// ==============================
//        process_callee_decl
// ==============================
void process_callee_decl(tree callee_decl, gimple* stmt) {

    if (!callee_decl || !stmt) return;
    // Keep the original safety check: we need both
    // -----------------------------------------------------------
    // A) Detect once whether this callee is directly recursive
    //    (i.e., its own body calls itself)
    // -----------------------------------------------------------
 

    //std::fprintf(stderr,
      //  "[DEBUG] Processing call to '%s' from '%s'\n",
      //  callee_name.c_str(), current_fn_name.c_str());

 //  Recursion: check if callee is a recursive transition and this is an entry call 
//
// g_recursive_method_names is a set<string> filled from validate_wcet_vs_timed_rules
// using the 'method' names from Timed_Typestate_Rules.

// is this call going to a recursive *transition*?
 if (!is_gimple_call(stmt)) {
        // handle other kinds of statements...
        return;
    }

   tree callee = gimple_call_fndecl(stmt);
    if (!callee || TREE_CODE(callee) != FUNCTION_DECL)
        return;
 //std::fprintf(stderr, "[pcd] process_callee_decl called\n");

    if (!callee_decl || TREE_CODE(callee_decl) != FUNCTION_DECL)
        return;

    const char* raw_name = IDENTIFIER_POINTER(DECL_NAME(callee_decl));
    std::string callee_name(raw_name);   // e.g. "Robot::AdjustActuators"

    // print every call we see at least once
   // std::fprintf(stderr, "[pcd]  saw call to '%s'\n", callee_name.c_str());

    // normalize to base name so it matches the typestate method "AdjustActuators"
    std::string base_name = callee_name;
    auto pos = base_name.rfind("::");
    if (pos != std::string::npos)
        base_name = base_name.substr(pos + 2);

    //std::fprintf(stderr, "[pcd]  base name = '%s'\n", base_name.c_str());

  


    
    {
        // 1) Basic prep like in the second version
        unsigned nargs = gimple_call_num_args(stmt);
        tree flagged_tmp = nullptr;
        tree first_obj = nullptr;
        tree dive_obj  = nullptr;

        // 2) Determine if it's a non-static member call
        bool is_member =
            DECL_CONTEXT(callee_decl) &&
            TREE_CODE(DECL_CONTEXT(callee_decl)) == RECORD_TYPE &&
            !DECL_STATIC_FUNCTION_P(callee_decl);

      
        if (nargs >= 1) {
            tree arg0 = gimple_call_arg(stmt, 0);
            if (arg0 && is_object_of_Flagged(arg0, flagged_tmp)) {
                first_obj = track_all_aliases(get_original_object(flagged_tmp));
            }
        }

        // 4) Decide the "dive object" policy (mirrors your second version)
        if (is_member) {
            // Member call: prefer a flagged 2nd argument (arg1) as the inter-proc anchor
            if (nargs >= 2) {
                tree arg1 = gimple_call_arg(stmt, 1);
                if (arg1 && is_object_of_Flagged(arg1, flagged_tmp)) {
                    dive_obj = track_all_aliases(get_original_object(flagged_tmp));
                }
            }
        } else {
            // Free/static call: fall back to first_obj (flagged arg0)
            dive_obj = first_obj;
        }

        // 5) If we found a dive_obj, build an analyzed-context key and possibly dive
        if (dive_obj) {
            // Compute current state for the dive object (or -1 if unknown)
            int in_state = -1;
            if (auto it = state_manager.object_states.find(dive_obj);
                it != state_manager.object_states.end())
            {
                in_state = it->second;
            }

            AnalysisContextKey key{
                callee_decl,
                dive_obj,
                in_state,
                "" // optional tag/channel; keep empty as in your second version
            };

            // Guard against re-entrancy / infinite recursion
            if (analyzed_contexts.insert(key).second) {
                // Build ParamMap (formal -> canonical actual), like your second version
                ParamMap p2a;
                // Iterate formals alongside indices
                tree formal = DECL_ARGUMENTS(callee_decl);
                for (unsigned idx = 0; idx < nargs && formal; ++idx, formal = TREE_CHAIN(formal)) {
                    tree actual = gimple_call_arg(stmt, idx);
                    tree canon  = actual ? track_all_aliases(get_original_object(actual)) : NULL_TREE;
                    p2a[formal] = canon;
                }

                // (Optional) If operator-> on a tracked "dive_obj", propagate alias (non-destructive)
                if (callee_name == "operator->") {
                    if (tree lhs = gimple_call_lhs(stmt)) {
                        state_manager.register_alias(lhs, dive_obj);
                    }
                }

                // If the callee has a body, we can perform an inter-procedural dive now.
                if (gimple_has_body_p(callee_decl)) {
                    if (function* fn = DECL_STRUCT_FUNCTION(callee_decl)) {
                        push_cfun(fn);
                        Analyse_gimple_in_function(std::move(p2a));
                        pop_cfun();
                    }
                }
             
            }
            
        }
    }
    // ----------------------------------------------------------------------
    //  "especially analyzed context" coverage
    // ----------------------------------------------------------------------

    //Original: member functions only 
    if (TREE_CODE(callee_decl) == FUNCTION_DECL &&
        DECL_CONTEXT(callee_decl) &&
        TREE_CODE(DECL_CONTEXT(callee_decl)) == RECORD_TYPE)
    {
        // class gate
        std::string class_name;
        if (tree cls = DECL_CONTEXT(callee_decl)) {
            if (TYPE_NAME(cls) && DECL_NAME(TYPE_NAME(cls))) {
                const char* cls_raw = IDENTIFIER_POINTER(DECL_NAME(TYPE_NAME(cls)));
                if (cls_raw) class_name = demangle(cls_raw);
            }
        }
        if (class_name != TypestateClassConnector_args[0]) goto PARAMS;

        // 'this'
        tree this_arg = NULL_TREE;
        if (!DECL_STATIC_FUNCTION_P(callee_decl))
            this_arg = gimple_call_arg(stmt, 0);
        if (!this_arg) goto PARAMS;

        // normalize object once
        this_arg = get_original_object(this_arg);
        this_arg = track_all_aliases(this_arg);
        if (!this_arg) goto PARAMS;

        // operator-> : alias only
        if (callee_name == "operator->") {
            if (tree lhs = gimple_call_lhs(stmt)) {
                state_manager.register_alias(lhs, this_arg);
            }
            goto PARAMS;
        }

        // ensure it's a tracked object (base or subtype)
        tree obj_address = NULL_TREE;
        if (!is_object_of_Flagged(this_arg, obj_address)) goto PARAMS;

        // ensure current state
        int current_state = 0;
        if (auto it = state_manager.object_states.find(obj_address);
            it != state_manager.object_states.end())
            current_state = it->second;
        else
            state_manager.object_states[obj_address] = current_state;

        // basic block / terminator
        basic_block bb = gimple_bb(stmt);
        gimple_stmt_iterator tail = gsi_last_bb(bb);
        gimple* term = gsi_end_p(tail) ? nullptr : gsi_stmt(tail);

        // decide guarded path
        bool returns_guarded = method_returns_boolean_or_enum(callee_decl);
        bool feeds_cond = call_feeds_terminator_cond(stmt, term);

        if (returns_guarded || feeds_cond) {
            // candidate next states from current_state for THIS method
            std::vector<int> candidates = transitions_for(current_state, callee_name, gimple_location(stmt));

            // bitset for merge infra
            Bitset outset;
            for (int s : candidates) if (0 <= s && s < (int)outset.size()) outset.set(s);

            //  Case A: this call feeds THIS BB's GIMPLE_COND
            if (call_feeds_terminator_cond(stmt, term)) {
                int ifid = get_if_else_id_for_terminator(term);
                auto& ctx = ensure_ifctx(ifid, term);

                // Candidates for THIS method
                std::vector<int> candidates2 =
                    transitions_for(current_state, callee_name, gimple_location(stmt));

                // Record for merge bookkeeping
                record_pre_state(ifid, obj_address, current_state);
                Bitset outset2; for (int s : candidates2) if (0 <= s && s < (int)outset2.size()) outset2.set(s);
                record_arm_out_set(ifid, obj_address, outset2);

                tree key = norm_obj_key(obj_address);
                if (!ctx.arm_choice.count(key)) {
                    auto two = pick_two_arms(candidates2);
                    tree res = gimple_call_lhs(stmt);

                    CondOp op; long long K; bool res_on_lhs; std::string K_name;
                    std::pair<int,int> choice;

                    if (extract_eq_ne_with_const(term, res, op, K, res_on_lhs, K_name)) {
                        if (op == OP_EQ) {
                            choice = (K == 0)
                                ? std::make_pair(two.second, two.first)
                                : std::make_pair(two.first,  two.second);
                        } else { // OP_NE
                            choice = (K == 0)
                                ? std::make_pair(two.second, two.first)
                                : std::make_pair(two.first,  two.second);
                        }
                    } else {
                        // Fallback
                        bool t_true = true_edge_means_call_true(term, stmt);
                        choice = t_true ? std::make_pair(two.first, two.second)
                                        : std::make_pair(two.second, two.first);
                        fprintf(stderr,
                            "Debug: Fallback plan. two={%d,%d}. PLAN TRUE→%d, FALSE→%d (ifid=%d).\n",
                            two.first, two.second, choice.first, choice.second, ifid);
                    }

                    ctx.arm_choice[key] = choice;
                }
                int next = -1; for (int i = 0; i < (int)outset.size(); ++i) if (outset.test(i)) { next = i; break; }
                state_manager.object_states[obj_address] = next;
                end_branch_processing(obj_address);
                return;
            }

            //  Case B: already in successor arm (use predecessor's COND & the GUARD call)
            {
                gimple* pred_term = controlling_terminator_of(bb); // the COND that led here
                int ifid = pred_term ? get_if_else_id_for_terminator(pred_term)
                                     : get_if_else_id_for_bb(bb);

                auto& ctx = ensure_ifctx(ifid, pred_term);

                // Record pre/outset from THIS call (for merge bookkeeping)
                std::vector<int> candidates2 =
                    transitions_for(current_state, callee_name, gimple_location(stmt));
                record_pre_state(ifid, obj_address, current_state);
                Bitset outset2; for (int s : candidates2) if (0 <= s && s < (int)outset2.size()) outset2.set(s);
                record_arm_out_set(ifid, obj_address, outset2);

                tree key = norm_obj_key(obj_address);
                if (!ctx.arm_choice.count(key)) {
                    // Find the GUARD call whose result the predecessor COND tested
                    gimple* guard_call = (pred_term && gimple_code(pred_term) == GIMPLE_COND)
                                       ? find_guard_call_for_cond(pred_term)
                                       : nullptr;

                    // Prefer guard method’s transitions; else fall back to current candidates
                    std::vector<int> seed_cands;
                    std::string guard_name = "<unknown>";
                    if (guard_call && is_gimple_call(guard_call)) {
                        if (tree gd = gimple_call_fndecl(guard_call)) {
                            if (DECL_NAME(gd)) guard_name = demangle(IDENTIFIER_POINTER(DECL_NAME(gd)));
                        }
                        seed_cands = transitions_for(current_state, guard_name, gimple_location(guard_call));
                    } else {
                        seed_cands = candidates2;
                    }

                    auto two = pick_two_arms(seed_cands);
                    std::pair<int,int> choice;

                    if (guard_call) {
                        // Read (==/!=) and K from the predecessor COND relative to GUARD result
                        tree res = gimple_call_lhs(guard_call);
                        CondOp op; long long K; bool res_on_lhs; std::string K_name;

                        if (extract_eq_ne_with_const(pred_term, res, op, K, res_on_lhs, K_name)) {
                            if (op == OP_EQ) {
                                choice = (K == 0)
                                    ? std::make_pair(two.second, two.first)
                                    : std::make_pair(two.first,  two.second);
                            } else { // OP_NE
                                choice = (K == 0)
                                    ? std::make_pair(two.first,  two.second)
                                    : std::make_pair(two.second, two.first);
                            }
                        } else {
                            // Fallback if pattern isn’t simple eq/ne with const
                            bool t_true = true_edge_means_call_true(pred_term, guard_call);
                            choice = t_true ? std::make_pair(two.first, two.second)
                                            : std::make_pair(two.second, two.first);
                            fprintf(stderr,
                                "Debug: GUARD fallback. two={%d,%d}. PLAN TRUE→%d, FALSE→%d (ifid=%d).\n",
                                two.first, two.second, choice.first, choice.second, ifid);
                        }
                    } else {
                        // No guard found — fallback using current stmt vs pred_term
                        bool t_true = true_edge_means_call_true(pred_term, stmt);
                        choice = t_true ? std::make_pair(two.first, two.second)
                                        : std::make_pair(two.second, two.first);
                        fprintf(stderr,
                            "Debug: No guard; fallback. two={%d,%d}. PLAN TRUE→%d, FALSE→%d (ifid=%d).\n",
                            two.first, two.second, choice.first, choice.second, ifid);
                    }

                    ctx.arm_choice[key] = choice;
                }
                end_branch_processing(key);
                int next = -1; for (int i = 0; i < (int)outset.size(); ++i) if (outset.test(i)) { next = i; break; }
                state_manager.object_states[obj_address] = next;
                return;
            }

            //  Case C: straight-line (no branch)
            if (outset.count() != 1) {
                error_at(gimple_location(stmt),
                         "Typestate: call '%s' yields multiple possible next states outside a branch.",
                         callee_name.c_str());
                return;
            }

            return;
        }
   

        //  NON-GUARDED path
        {
            // If this BB belongs to an if/switch arm, always defer (never straight-line inside an arm)
            int branch_id = get_if_else_id_for_bb(bb);
            if (branch_id >= 0) {
                defer_branch_handling(branch_id, obj_address, callee_name, gimple_location(stmt), stmt);
                return;
            }

            // No branch context → straight-line check
            state_manager.Typestate_Checking(obj_address, callee_name, gimple_location(stmt));
               if (!is_recursive_transition_method(base_name)) {
        return;  // not a recursive FSM edge → nothing to do
    }

   { std::fprintf(stderr,
        "[pcd]  '%s' is a recursive transition method, checking depth\n",
        base_name.c_str());

    // Enforce: first argument = recursion depth (constant)
    unsigned nargs = gimple_call_num_args(stmt);
    if (nargs < 1) {
        error_at(gimple_location(stmt),
                 "Call to recursive transition '%s' must specify a recursion "
                 "depth in its first parameter.",
                 base_name.c_str());
        return;
    }

    tree arg0 = gimple_call_arg(stmt, 1);
    if (!arg0 || TREE_CODE(arg0) != INTEGER_CST) {
        error_at(gimple_location(stmt),
                 "Call to recursive transition '%s' must use a compile-time "
                 "constant depth in its first argument (e.g., %s(2)).",
                 base_name.c_str(), base_name.c_str());
        return;
    }

    long depth = (long)TREE_INT_CST_LOW(arg0);
    if (depth <= 0) {
        error_at(gimple_location(stmt),
                 "Recursion depth for '%s' must be positive (got %ld).",
                 base_name.c_str(), depth);
        return;
    }

    // Store it for WCET side
    auto it = g_recursion_bound_by_method.find(base_name);
    if (it != g_recursion_bound_by_method.end()) {
        if (it->second != (int)depth) {
            error_at(gimple_location(stmt),
                     "Inconsistent recursion depth for '%s': previously %d, now %ld.",
                     base_name.c_str(), it->second, depth);
            return;
        }
    } else {
        g_recursion_bound_by_method[base_name] = (int)depth;
    }

    std::fprintf(stderr,
        "[pcd]  recursion depth for '%s' set to %ld\n",
        base_name.c_str(), depth);
        depth_r=depth;

 return;
    }

    
            return;
        }
    }

    //  Arm-finalization hook
    {
        tree this_arg = NULL_TREE;
        if (!DECL_STATIC_FUNCTION_P(callee_decl))
            this_arg = gimple_call_arg(stmt, 0);
        if (!this_arg) goto PARAMS;

        this_arg = get_original_object(this_arg);
        this_arg = track_all_aliases(this_arg);
        if (!this_arg) goto PARAMS;
        tree obj_address = NULL_TREE;
        if (!is_object_of_Flagged(this_arg, obj_address)) goto PARAMS;
        basic_block bb_here = gimple_bb(stmt);
        if (bb_here && is_inside_if_or_switch(bb_here) && is_end_of_arm_bb(bb_here)) {
            tree key = norm_obj_key(obj_address);
            if (key) {
                end_branch_processing(key);
            }
        }
        return;
    }


        

PARAMS:
    //  Parameter → argument alias propagation (original)
    if (TREE_CODE(callee_decl) == FUNCTION_DECL) {
        tree param = DECL_ARGUMENTS(callee_decl);
        unsigned int arg_count = gimple_call_num_args(stmt);
        unsigned int i = 0;
        for (; param; param = TREE_CHAIN(param), ++i) {
            tree arg = (i < arg_count) ? gimple_call_arg(stmt, i) : NULL_TREE;
            if (arg) { arg = get_original_object(arg); arg = track_all_aliases(arg); }

            tree obj_address = NULL_TREE;
            if (arg && is_object_of_Flagged(arg, obj_address)) {
                state_manager.register_alias(param, obj_address);
                if (DECL_STRUCT_FUNCTION(callee_decl)) {
                    process_function_body(callee_decl, obj_address);
                }
            }
        }
    }
}


void check_recursive_wcet_for_function(tree fn_decl, double wcet_per_call_ms, double time_guard_ms) {
    int bound = 0;
    if (!get_recursion_bound(fn_decl, bound)) {
        // Recursive but no bound recorded → reject
        error_at(UNKNOWN_LOCATION,
                 "Recursive function '%s' lacks a compile-time recursion bound.",
                 IDENTIFIER_POINTER(DECL_NAME(fn_decl)));
        fatal_error(UNKNOWN_LOCATION, "Unbounded recursion is not time-safe.");
    }

    double total_ms = bound * wcet_per_call_ms;
    if (total_ms > time_guard_ms) {
        error_at(UNKNOWN_LOCATION,
                 "Recursion in '%s' exceeds time guard: %g ms (bound=%d, per-call=%g ms) > %g ms.",
                 IDENTIFIER_POINTER(DECL_NAME(fn_decl)),
                 total_ms, bound, wcet_per_call_ms, time_guard_ms);
        fatal_error(UNKNOWN_LOCATION, "Recursive WCET violates time guard.");
    }
}


/*******************************
 * GIMPLE Statement Analysis
 *******************************/

 static const char* g_mark_file = nullptr;
void Analyse_gimple_statement(gimple* stmt, gimple_stmt_iterator* gsi)
{
    if (!stmt) return;

    
    if (is_gimple_assign(stmt))
    {
        tree L = gimple_assign_lhs(stmt);
        tree R = gimple_assign_rhs1(stmt);

        // ptr = &obj or T* p = &obj
        if (TREE_CODE(R) == ADDR_EXPR)
        {
            tree target  = TREE_OPERAND(R, 0);
            tree obj_addr = nullptr;
            if (is_object_of_Flagged(target, obj_addr))
            {
                tree lhs_root = get_original_object(L);
                state_manager.register_alias(lhs_root, obj_addr);
            }
        }

        // reference-type assignment T& r = obj;
        if (TREE_TYPE(L)
            && TREE_CODE(TREE_TYPE(L)) == REFERENCE_TYPE
            && TREE_CODE(R) == ADDR_EXPR)
        {
            tree target  = TREE_OPERAND(R, 0);
            tree obj_addr = nullptr;
            if (is_object_of_Flagged(target, obj_addr))
            {
                tree lhs_root = get_original_object(L);
                state_manager.register_alias(lhs_root, obj_addr);
            }
        }

        // object-to-object aliasing x = y
        tree Lobj = nullptr, Robj = nullptr;
        if (is_object_of_Flagged(L, Lobj) && is_object_of_Flagged(R, Robj))
        {
            state_manager.register_alias(Lobj, Robj);
        }
        else if (is_object_of_Flagged(R, Robj))
        {
            state_manager.register_alias(L, Robj);
        }

        // operator-> in an assignment (smart-pointer style)
        if (gimple_assign_rhs_code(stmt) == CALL_EXPR)
        {
            tree call   = gimple_assign_rhs1(stmt);
            tree callee = CALL_EXPR_FN(call);
            if (callee && TREE_CODE(callee) == FUNCTION_DECL)
            {
                const char* raw = IDENTIFIER_POINTER(DECL_NAME(callee));
                if (raw && std::string(raw) == "operator->")
                {
                    tree this_arg = CALL_EXPR_ARG(call, 0);
                    this_arg = track_all_aliases(get_original_object(this_arg));
                    state_manager.register_alias(L, this_arg);
                }
            }
        }

        return;
    }

    
    if (!is_gimple_call(stmt)) return;

    tree callee_decl = gimple_call_fndecl(stmt);
    if (!callee_decl) return;

    // For diagnostics: method name (unqualified)
    const char* nm = IDENTIFIER_POINTER(DECL_NAME(callee_decl));
    std::string method = nm ? demangle(nm) : "unknown_method";

    // Flag to detect if this call touches any flagged object
    tree flagged_obj = nullptr;
    tree tmp = nullptr;

    // 2a) Member call? Check 'this' first
    bool is_member = (DECL_CONTEXT(callee_decl)
                      && TREE_CODE(DECL_CONTEXT(callee_decl)) == RECORD_TYPE
                      && !DECL_STATIC_FUNCTION_P(callee_decl));

    if (is_member)
    {
        tree this_arg = gimple_call_arg(stmt, 0);
        if (this_arg)
        {
            // --- If 'this' is PHI, warn and skip checking ---
            if (TREE_CODE(this_arg) == SSA_NAME)
            {
                gimple* def = SSA_NAME_DEF_STMT(this_arg);
                if (def && gimple_code(def) == GIMPLE_PHI)
                {
                      error_at(gimple_location(stmt),
                               "Typestate: receiver is a PHI for %qs; skipping check",
                               method.c_str());
                            // after warning_at(...):
if (g_mark_file) {
    if (FILE* f = std::fopen(g_mark_file, "ab")) {
        std::fputs("phi\n", f);
        std::fclose(f);
    }
}

                    return;  // don't do any typestate checking for this call
                }
            }

            // else: see if it’s a flagged object to trigger normal processing
            if (is_object_of_Flagged(this_arg, tmp))
                flagged_obj = track_all_aliases(get_original_object(tmp));
        }
    }

    // 2b) If not a member (or 'this' wasn’t flagged), scan explicit args
    if (!flagged_obj)
    {
        unsigned nargs = gimple_call_num_args(stmt);
        for (unsigned i = 0; i < nargs; ++i)
        {
            tree arg = gimple_call_arg(stmt, i);
            if (!arg) continue;

            //  If an argument is PHI 
            if (TREE_CODE(arg) == SSA_NAME)
            {
                gimple* def = SSA_NAME_DEF_STMT(arg);
                if (def && gimple_code(def) == GIMPLE_PHI)
                {
                    // Check if this PHI-arg is the flagged object we care about
                    if (is_object_of_Flagged(arg, tmp))
                    {
                        warning_at(gimple_location(stmt), 0,
                                   "Typestate: argument is a PHI for %qs; skipping check",
                                   method.c_str());
                        return;  // skip checking this call entirely
                    }
                }
            }

            // Otherwise, if a plain flagged object appears, mark it
            if (!flagged_obj && is_object_of_Flagged(arg, tmp))
            {
                flagged_obj = track_all_aliases(get_original_object(tmp));
            }
        }
    }

    // 3) If no flagged object is touched, nothing to do
    if (!flagged_obj) return;

    // 4) Proceed with your existing static processing (no runtime)
    process_callee_decl(callee_decl, stmt);
}



/*───────────────────────────────────────────────────────────────────────────*/
/*                                                          */
/*    – seed the work-queue with every function that has a body             */
/*    – pull items, push cfun, run Analyse_gimple_in_function, pop cfun     */
/*───────────────────────────────────────────────────────────────────────────*/
void Analyse_all_functions()
{

     
    // ── 1) Find and seed only 'main' 
    tree main_decl = nullptr;
    cgraph_node *cgn = nullptr;
    FOR_EACH_DEFINED_FUNCTION(cgn)
    {
        tree d = cgn->decl;
        if (DECL_NAME(d) &&
            strcmp(IDENTIFIER_POINTER(DECL_NAME(d)), "main") == 0 &&
            gimple_has_body_p(d))
        {
            main_decl = d;
            break;
        }
    }
   harvest_annot_from_decl(main_decl);
    if (!main_decl)
        return;  

    // Seed the work‐queue with main, 
    AnalysisContextKey root_ctx{
        /* fn_decl    */ main_decl,
        /* obj        */ nullptr,
        /* state      */ 0,
        /* alias_head */ ""
    };
    work_queue.emplace(main_decl, ParamMap{}, root_ctx);

    //  2) Standard work‐loop 
    while (!work_queue.empty())
    {
        auto [fn_decl, param_map, ctx] = work_queue.front();
        work_queue.pop();

        if (!fn_decl || !gimple_has_body_p(fn_decl))
            continue;

        function *fn = DECL_STRUCT_FUNCTION(fn_decl);
        if (!fn)
            continue;

        push_cfun(fn);
        Analyse_gimple_in_function(param_map);
        pop_cfun();
    }

}

static void on_finish_parse_fn(void* gcc_data, void*) {
    tree fndecl = (tree)gcc_data;
    harvest_annot_from_decl(fndecl);
 
}

static void on_finish_decl(void* gcc_data, void*) {
    tree decl = (tree)gcc_data;
    if (!decl) return;
    if (TREE_CODE(decl) == FUNCTION_DECL) {
        harvest_annot_from_decl(decl);
    }
   
}





static inline bool is_real_user_fn_def(tree fndecl) {
    if (!fndecl || TREE_CODE(fndecl) != FUNCTION_DECL) return false;

    // Must have a body attached (parsed definition)
    if (!DECL_STRUCT_FUNCTION(fndecl)) return false;          // body not built
    if (!DECL_INITIAL(fndecl)) return false;                  // still a decl only

    // Skip synthesized & system
    if (DECL_ARTIFICIAL(fndecl)) return false;
    location_t loc = DECL_SOURCE_LOCATION(fndecl);
    if (!loc) return false;
    if (in_system_header_at(loc)) return false;

    // Must come from a file we can print
    const char* file = LOCATION_FILE(loc);
    if (!file || !*file) return false;

    return true;
}
static void wcet_on_pre_genericize(void*, void*) {
    if (!cfun || !cfun->decl) return;
    tree fndecl = cfun->decl;

    // your usual guards...
    if (DECL_ARTIFICIAL(fndecl)) return;
    location_t loc = DECL_SOURCE_LOCATION(fndecl);
    if (!loc || in_system_header_at(loc)) return;
    if (g_wcet_pending.empty()) return;

    std::string payload = std::move(g_wcet_pending.front());
    g_wcet_pending.pop_front();

    AnnotRow tmp;
    if (!parse_payload(payload.c_str(), tmp)) return;

    const char* name = DECL_NAME(fndecl) ? IDENTIFIER_POINTER(DECL_NAME(fndecl)) : "";
    std::string method = name ? name : "";

    
    record_wcet_row(/*class_key_canon=*/nullptr, method, fndecl, tmp.wcet_ms);
    
}



static void wcet_on_finish_parse_function(void* gcc_fn, void* /*user*/) {
    // Try to get the fndecl robustly across GCC versions
    tree fndecl = nullptr;

    // Preferred: from event payload (struct function*)
    if (gcc_fn) {
        struct function* fun = (struct function*)gcc_fn;
        if (fun && fun->decl)
            fndecl = fun->decl;
    }
    // Fallback: global
    if (!fndecl && current_function_decl)
        fndecl = current_function_decl;

    if (!fndecl || TREE_CODE(fndecl) != FUNCTION_DECL) return;
    if (is_from_system_header(fndecl)) return;
    if (is_artificial(fndecl)) return;
    if (g_wcet_pending.empty()) return;

    // Take the next queued pragma payload and parse it
    std::string payload = std::move(g_wcet_pending.front());
    g_wcet_pending.pop_front();

    AnnotRow row;
    if (!parse_payload(payload.c_str(), row)) {
        const char* nm = DECL_NAME(fndecl) ? IDENTIFIER_POINTER(DECL_NAME(fndecl)) : "<noname>";
        warning(0, "malformed wcet payload for %qs: %qs", nm, payload.c_str());
        return;
    }

    // Fill in name/location, store and echo
    row.qualified = qualified_name_for_decl(fndecl);
    location_t loc = DECL_SOURCE_LOCATION(fndecl);
    row.file = LOCATION_FILE(loc) ? LOCATION_FILE(loc) : "";
    row.line = LOCATION_LINE(loc);
    row.method = DECL_NAME(fndecl) ? IDENTIFIER_POINTER(DECL_NAME(fndecl)) : "";

    g_rows.push_back(row);
  //  fprintf(stderr, "[wcet] attached %.6f ms to %s @ %s:%d\n",
     //       row.wcet_ms, row.qualified.c_str(), row.file.c_str(), row.line);

    // Optional: also populate the class->method map
    tree ctx = DECL_CONTEXT(fndecl);
    tree class_type = (ctx && TREE_CODE(ctx) == RECORD_TYPE) ? TYPE_MAIN_VARIANT(ctx) : nullptr;
    const void* class_key_canon = (const void*)class_type;
    g_annot_wcet_ms[class_key_canon][row.method] = row.wcet_ms;

    const char* nm = DECL_NAME(fndecl) ? IDENTIFIER_POINTER(DECL_NAME(fndecl)) : "<noname>";
//fprintf(stderr, "[wcet] finish-parse fn=%s (pending=%zu)\n", nm, g_wcet_pending.size());


}

int plugin_init(struct plugin_name_args* plugin_info, struct plugin_gcc_version* version) {
    const char* plugin_name = plugin_info->base_name;
    (void)version;

register_callback(plugin_name, PLUGIN_START_UNIT,
  +[](void*, void*) { fprintf(stderr, "[coconut] plugin loaded\n"); }, nullptr);

   


    register_callback(plugin_name, PLUGIN_PRE_GENERICIZE, on_template_instantiation, nullptr);

    register_callback(plugin_name, PLUGIN_ALL_PASSES_START, [](void*, void*) {
        static bool once = false;
        if (!once) {
            try_merge_fsms_respecting_lsp_for_class_pair();
             validate_wcet_vs_timed_rules();
            Analyse_all_functions();
            print_timed_typestate_rules();
         
        
            once = true;
        }
    }, nullptr);

    




           register_callback(plugin_name, PLUGIN_FINISH_DECL, wcet_on_finish_decl_wcet, nullptr);
register_callback(plugin_name, PLUGIN_PRE_GENERICIZE, wcet_on_pre_genericize, nullptr);


  
    register_callback(plugin_name, PLUGIN_FINISH_UNIT,
                  +[](void*, void*) {
                      harvest_all_functions();
             //         print_annotated_wcet_list();
                 //   validate_wcet_vs_timed_rules();
                      
                  }, nullptr);
    
    return 0;
}
