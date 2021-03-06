#include "hba_printer.hpp"
#include <boost/foreach.hpp>

using namespace hexabus;


void hexabus::tab(int indent) {
  for (int i = 0; i < indent; ++i)
	std::cout << ' ';
}

struct hba_node_printer : boost::static_visitor<> {
  hba_node_printer(int indent = 0)
	: indent(indent)
  { }

	void operator()(if_clause_doc const& clause) const {
		tab(indent);
		std::cout << "if clause (line " << clause.lineno << "): "
			<< clause.name << std::endl;
		tab(tabsize);
		std::cout << " set eid " << clause.eid 
			<< " := " << clause.value << std::endl;
		tab(tabsize);
		std::cout << " goodstate: " << clause.goodstate << std::endl;
		tab(tabsize);
		std::cout << " badstate: " << clause.badstate << std::endl;
		tab(indent);
	}

	void operator()(state_doc const& hba) const
	{
		tab(indent);
		std::cout << "state: " << hba.name << " (line "
			<< hba.lineno << ")" << std::endl;
		tab(indent);
		std::cout << '{' << std::endl;

		BOOST_FOREACH(if_clause_doc const& if_clause, hba.if_clauses)
		{
			hba_node_printer p(indent);
			p(if_clause);
			//boost::apply_visitor(hba_node_printer(indent), if_clause);
		}

		tab(indent);
		std::cout << '}' << std::endl;

		tab(indent);
	}

	void operator()(condition_doc const& hba) const
	{
		tab(indent);
		std::cout << "condition (line " << hba.lineno << "): " << hba.name << std::endl;
		tab(tabsize);
		hba_node_printer p(indent);
    if(hba.cond.which() == 0)
      p(boost::get<cond_eidvalue_doc>(hba.cond));
    else if (hba.cond.which() == 1)
      p(boost::get<cond_timeout_doc>(hba.cond));
    else
      p(boost::get<cond_datetime_doc>(hba.cond));
	}

  void operator()(cond_eidvalue_doc const& hba) const
  {
    tab(indent);
    std::cout << "EID: " << hba.eid << std::endl;
    tab(tabsize);
    std::cout << "Operator: " << hba.op << std::endl;
    tab(tabsize);
    std::cout << "Value: " << hba.value << std::endl;
    tab(indent);
  }

  void operator()(cond_timeout_doc const& hba) const
  {
    tab(indent);
    std::cout << "Timeout: " << hba.value << std::endl;
    tab(indent);
  }

  void operator()(cond_datetime_doc const& hba) const
  {
    // TODO TODO
  }

	void operator()(std::string const& text) const
	{
		tab(indent+tabsize);
		std::cout << "text: \"" << text << '"' << std::endl;
	}

  int indent;
};

void hba_printer::operator()(hba_doc const& hba) const
{
  tab(indent);
  std::cout << "start state: " << hba.start_state << std::endl;
  tab(indent);
  std::cout << '{' << std::endl;

  BOOST_FOREACH(hba_doc_block const& block, hba.blocks)
  {
	//    mini_hba_node_printer p(indent);
	//    p(block);
	boost::apply_visitor(hba_node_printer(indent), block);
  }

  tab(indent);
  std::cout << '}' << std::endl;
}

