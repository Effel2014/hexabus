#ifndef LIBHBC_HBCOMP_GRAMMAR
#define LIBHBC_HBCOMP_GRAMMAR

#include <boost/config/warning_disable.hpp>
#include <boost/spirit/home/phoenix.hpp>
#include <boost/spirit/home/phoenix/statement/sequence.hpp>
#include <boost/spirit/home/qi.hpp>
#include <boost/spirit/home/support/info.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/support_multi_pass.hpp>
#include <boost/variant/recursive_variant.hpp>

#include <libhbc/file_pos.hpp>
#include <libhbc/hbc_enums.hpp>

namespace qi = boost::spirit::qi;
namespace classic = boost::spirit::classic;

typedef std::istreambuf_iterator<char> base_iterator_type;
typedef boost::spirit::multi_pass<base_iterator_type> forward_iterator_type;
typedef classic::position_iterator2<forward_iterator_type> pos_iterator_type;

namespace hexabus {

  struct error_traceback_t
  {
    error_traceback_t (std::string const & msg)
    {
      stack.push_back(msg);
    }

    template <typename ArgsT, typename ContextT, typename RT>
      void operator() (ArgsT, ContextT, RT) const
    { }

    static std::vector<std::string> stack;
  };
  std::vector<std::string> error_traceback_t::stack;

  struct FileState
  {
    FileState() : line(1)
    { }

    int line;
  };

  struct UpdateFileInfo
  {
    UpdateFileInfo(FileState& state) : m_state(state)
    {}
    FileState& m_state;
  };

  // Grammar definition
  template <typename Iterator>
    struct hexabus_comp_grammar : public qi::grammar<Iterator, hbc_doc(), skipper<Iterator> >
  {
    typedef skipper<Iterator> Skip;

    hexabus_comp_grammar(std::vector<std::string>& error_hints,
      pos_iterator_type& current_position) : hexabus_comp_grammar::base_type(start),
        _error_hints(error_hints), _current_position(current_position)
    {
      using qi::lit;
      using qi::lexeme;
      using qi::on_error;
      using qi::rethrow;
      using qi::fail;
      using qi::accept;
      using qi::repeat;
      using boost::spirit::ascii::char_;
      using qi::uint_;
      using qi::float_;
      using boost::spirit::ascii::string;
      using boost::spirit::ascii::space;
      using namespace qi::labels;
      using boost::spirit::_1;
      using boost::spirit::_2;
      using boost::spirit::_3;
      using boost::spirit::_4;
      using boost::spirit::eps;
      using boost::spirit::eoi;
      using boost::spirit::no_skip;
      using boost::spirit::repository::qi::file_pos;

      using boost::phoenix::construct;
      using boost::phoenix::val;
      using boost::phoenix::ref;
      using boost::phoenix::bind;
      using boost::phoenix::let;

      // Assignment, constants, ...
      is = eps > lit(":=");
      constant = placeholder | float_ | (lit("i") > uint_) | lit("true") | lit("false");

      // Basic elements: Identifier, assignment, ...
      identifier %= char_("a-zA-Z_") > *char_("a-zA-Z0-9_");
      placeholder %= char_("$") > file_pos > *char_("a-zA-Z0-9_");
      filename %= eps > lexeme[char_("a-zA-Z0-9_/") > *char_("a-zA-Z0-9_./")];
      on_error<rethrow>(identifier, error_traceback_t("Invalid identifier"));
      // device_name.endpoint_name
      global_endpoint_id %= ( placeholder | identifier ) > '.' > file_pos > ( identifier | placeholder );
      ipv6_address %= +( char_("a-fA-F0-9:"));
      datatype = ( lit("BOOL")[_val = DT_BOOL] | lit("UINT8")[_val = DT_UINT8] | lit("UINT32")[_val = DT_UINT32] | lit("FLOAT")[_val = DT_FLOAT] );
      access_lv = ( lit("read")[_val = AC_READ] | lit("write")[_val = AC_WRITE] | lit("broadcast")[_val = AC_BROADCAST] );

      // larger blocks: Aliases, state machines
      include %= lit("include") >> file_pos > filename > ';';

      ep_eid %= lit("eid") > uint_ > ';';
      ep_datatype %= lit("datatype") > datatype > ';';
      ep_access %= lit("access") > '{' > access_lv > *(',' > access_lv) > '}';
      endpoint_cmd = ( ep_eid | ep_datatype | ep_access );
      endpoint %= lit("endpoint") >> file_pos > identifier > '{' > *endpoint_cmd > '}';

      eid_list %= uint_ > *(',' > uint_);
      alias_ip %= lit("ip") > ipv6_address > ';';
      alias_eids %= lit("eids") > '{' > -eid_list > '}';
      alias_cmd = ( alias_ip | alias_eids );
      alias %= lit("device") >> file_pos > identifier > '{'
        > *alias_cmd > '}';

      stateset %= lit("states") > file_pos > '{' > identifier > *(',' > identifier) > '}';
      statemachine %= lit("machine") > file_pos > identifier > '{' > stateset > *in_clause > '}';

      in_clause %= lit("in") >> file_pos > '(' > identifier > ')'
        > '{' > (always_clause | *if_clause) > '}';

      bool_op %= ( lit("||")[_val = OR] | lit("&&")[_val = AND] );
      comp_op %= ( lit("==")[_val = STM_COMP_EQ] | lit("<=")[_val = STM_COMP_LEQ] | lit(">=")[_val = STM_COMP_GEQ] | lit("<")[_val = STM_COMP_LT] | lit(">")[_val = STM_COMP_GT] | lit("!=")[_val = STM_COMP_NEQ] );
      atomic_condition %= lit("ep") > global_endpoint_id > comp_op > constant;
      timeout_condition %= lit("timeout") > constant;
      timer_condition %= lit("time") > time_f > time_comp_op > constant;
      time_f %= ( lit("hour")[ _val = TF_HOUR ] | lit("minute")[ _val = TF_MINUTE ] | lit("second")[ _val = TF_SECOND ] | lit("day")[ _val = TF_DAY ]
                       | lit("month")[ _val = TF_MONTH] | lit("year")[ _val = TF_YEAR ] | lit("weekday")[ _val = TF_WEEKDAY ] );
      time_comp_op %= ( lit(">")[ _val = STM_COMP_GT ] | lit("<")[ _val = STM_COMP_LT ] );
      compound_condition %= '(' > condition > ')' > bool_op > '(' > condition > ')';
      tautology %= lit("true")[_val = 1];
      always_tautology %= eps[_val = 1];
      condition %= ( tautology | atomic_condition | timeout_condition | timer_condition | compound_condition );
      command_block %= *command > goto_command > ';';
      guarded_command_block %= '(' > condition > ')' > '{' > command_block > '}';
      always_command_block %= lit("always") >> always_tautology > '{' > command_block > '}';

      if_clause %= ( lit("if") >> file_pos > guarded_command_block
        > *( lit("else") > lit("if") > guarded_command_block ) );
      always_clause %= file_pos >> always_command_block;

      // module definitions
      placeholder_list %= placeholder > *( ',' > placeholder );
      module %= lit("module") >> file_pos > identifier
        > '(' > -placeholder_list > ')'
        > '{' > stateset > *in_clause > '}';

      // module instantiations
      inst_parameter %= ( constant | identifier );
      instantiation %= lit("instance") >> file_pos > identifier > ':' > identifier > '(' > -(inst_parameter > *(',' > inst_parameter)) > ')' > ';';

      // commands that "do something"
      command %= write_command > ';'; // more commands can be added here, if needes
      write_command %= lit("write") >> file_pos > global_endpoint_id > is > constant;
      goto_command %= lit("goto") >> file_pos > identifier;

      start %= *( include | endpoint | alias | statemachine | module | instantiation ) > eoi;

      start.name("toplevel");
      identifier.name("identifier");
      filename.name("filename");
      alias.name("alias");
      statemachine.name("statemachine");
      condition.name("condition");
      command.name("command");
      goto_command.name("goto command");

      on_error<rethrow> (start, error_traceback_t("Cannot parse input"));
    }

    qi::rule<Iterator, void(), Skip> is;
    qi::rule<Iterator, constant_doc(), Skip> constant;
    qi::rule<Iterator, std::string()> identifier;
    qi::rule<Iterator, std::string(), Skip> filename;
    qi::rule<Iterator, int(), Skip> datatype;
    qi::rule<Iterator, access_level(), Skip> access_lv;
    qi::rule<Iterator, ep_eid_doc(), Skip> ep_eid;
    qi::rule<Iterator, ep_datatype_doc(), Skip> ep_datatype;
    qi::rule<Iterator, ep_access_doc(), Skip> ep_access;
    qi::rule<Iterator, endpoint_cmd_doc(), Skip> endpoint_cmd;
    qi::rule<Iterator, endpoint_doc(), Skip> endpoint;
    qi::rule<Iterator, global_endpoint_id_doc()> global_endpoint_id;
    qi::rule<Iterator, std::string(), Skip> ipv6_address;
    qi::rule<Iterator, std::string(), Skip> ipv6_address_block;
    qi::rule<Iterator, include_doc(), Skip> include;
    qi::rule<Iterator, alias_ip_doc(), Skip> alias_ip;
    qi::rule<Iterator, alias_eids_doc(), Skip> alias_eids;
    qi::rule<Iterator, alias_cmd_doc(), Skip> alias_cmd;
    qi::rule<Iterator, eid_list_doc(), Skip> eid_list;
    qi::rule<Iterator, alias_doc(), Skip> alias;
    qi::rule<Iterator, stateset_doc(), Skip> stateset;
    qi::rule<Iterator, statemachine_doc(), Skip> statemachine;
    qi::rule<Iterator, in_clause_doc(), Skip> in_clause;
    qi::rule<Iterator, int(), Skip> comp_op;
    qi::rule<Iterator, atomic_condition_doc(), Skip> atomic_condition;
    qi::rule<Iterator, timeout_condition_doc(), Skip> timeout_condition;
    qi::rule<Iterator, timer_condition_doc(), Skip> timer_condition;
    qi::rule<Iterator, time_fields(), Skip> time_f;
    qi::rule<Iterator, comp_operator(), Skip> time_comp_op;
    qi::rule<Iterator, compound_condition_doc(), Skip> compound_condition;
    qi::rule<Iterator, unsigned int(), Skip> tautology;
    qi::rule<Iterator, unsigned int(), Skip> always_tautology;
    qi::rule<Iterator, condition_doc(), Skip> condition;
    qi::rule<Iterator, command_block_doc(), Skip> command_block;
    qi::rule<Iterator, guarded_command_block_doc(), Skip> guarded_command_block;
    qi::rule<Iterator, guarded_command_block_doc(), Skip> always_command_block;
    qi::rule<Iterator, bool_operator(), Skip> bool_op;
    qi::rule<Iterator, int(), Skip> else_lit;
    qi::rule<Iterator, if_clause_doc(), Skip> if_clause;
    qi::rule<Iterator, if_clause_doc(), Skip> always_clause;
    qi::rule<Iterator, placeholder_list_doc(), Skip> placeholder_list;
    qi::rule<Iterator, module_doc(), Skip> module;
    qi::rule<Iterator, placeholder_doc()> placeholder;
    qi::rule<Iterator, inst_parameter_doc(), Skip> inst_parameter;
    qi::rule<Iterator, instantiation_doc(), Skip> instantiation;
    qi::rule<Iterator, command_doc(), Skip> command;
    qi::rule<Iterator, write_command_doc(), Skip> write_command;
    qi::rule<Iterator, goto_command_doc(), Skip> goto_command;
    qi::rule<Iterator, hbc_doc(), Skip> start;

    std::vector<std::string>& _error_hints;
    pos_iterator_type& _current_position;

    FileState file_state;
  };
};

#endif // LIBHBC_HBCOMP_GRAMMAR

