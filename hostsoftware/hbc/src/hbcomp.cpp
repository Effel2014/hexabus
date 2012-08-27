#include <libhbc/common.hpp>
#include <libhbc/ast_datatypes.hpp>
#include <libhbc/skipper.hpp>
#include <libhbc/hbcomp_grammar.hpp>
#include <libhbc/hbc_printer.hpp>
#include <libhbc/graph_builder.hpp>
#include <libhbc/table_builder.hpp>

// commandline parsing.
#include <boost/program_options.hpp>
#include <boost/program_options/positional_options.hpp>
namespace po = boost::program_options;

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>

int main(int argc, char** argv)
{
  std::ostringstream oss;
  oss << "Usage: " << argv[0] << " [-i] inputfile [options]";
  po::options_description desc(oss.str());
  desc.add_options()
    ("help,h", "produce help message")
    ("input,i", po::value<std::string>(), "the input file")
    ("print,p", "print parsed version of the input file")
    ("graph,g", po::value<std::string>(), "output the program graph in graphviz format")
    ("tables,t", "build endpoint and alias tables") // TODO this has to happen automatically later.
  ;
  po::positional_options_description p;
  p.add("input", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  std::string infile;
  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  // TODO Version!
  if(!vm.count("input"))
  {
    std::cerr << "Error: You must specify an input file." << std::endl;
    return 1;
  } else {
    infile = vm["input"].as<std::string>();
  }

  std::vector<std::string> filenames; // vector to store the names of the files yet to be read
  filenames.push_back(infile); // add the file the user specified -- more files are added when "include"s are parsed

  hexabus::hbc_doc ast; // The AST - all the files get parsed into one ast
  bool okay = true;

  for(int f = 0; f < filenames.size() && okay; f++) {
    bool r = false;
    std::ifstream in(filenames[f].c_str(), std::ios_base::in);

    if(!in)
    {
      std::cerr << "Error: Could not open input file: " << filenames[f] << std::endl;
      // TODO if this is from an include, we could put the line number here
      return 1;
    }

    in.unsetf(std::ios::skipws); // no white space skipping, this will be handled by the parser

    typedef std::istreambuf_iterator<char> base_iterator_type;
    typedef boost::spirit::multi_pass<base_iterator_type> forward_iterator_type;
    typedef classic::position_iterator2<forward_iterator_type> pos_iterator_type;

    base_iterator_type in_begin(in);
    forward_iterator_type fwd_begin = boost::spirit::make_default_multi_pass(in_begin);
    forward_iterator_type fwd_end;
    pos_iterator_type position_begin(fwd_begin, fwd_end, filenames[f]);
    pos_iterator_type position_end;

    std::vector<std::string> error_hints;
    typedef hexabus::hexabus_comp_grammar<pos_iterator_type> hexabus_comp_grammar;
    typedef hexabus::skipper<pos_iterator_type> Skip;
    hexabus_comp_grammar grammar(error_hints, position_begin);
    Skip skipper;

    using boost::spirit::ascii::space;
    using boost::spirit::ascii::char_;

    try {
      r = phrase_parse(position_begin, position_end, grammar, skipper, ast);
    } catch(const qi::expectation_failure<pos_iterator_type>& e) {
      const classic::file_position_base<std::string>& pos = e.first.get_position();
      std::cerr << "Error in " << pos.file << " line " << pos.line << " column " << pos.column << std::endl
        << "'" << e.first.get_currentline() << "'" << std::endl
        << std::setw(pos.column) << " " << "^-- " << *hexabus::error_traceback_t::stack.begin() << std::endl;
    } catch(const std::runtime_error& e) {
      std::cout << "Exception occured: " << e.what() << std::endl;
    }

    if(r && position_begin == position_end) {
      std::cout << "Parsing of file " << filenames[f] << " succeeded." << std::endl;
      
      // Find includes and add them to file name list
      BOOST_FOREACH(hexabus::hbc_block block, ast.blocks) {
        if(block.which() == 0) { // include_doc
          // only add if filename does not already exist
          bool exists = false;
          BOOST_FOREACH(std::string fn, filenames) {
            if(fn == boost::get<hexabus::include_doc>(block).filename)
              exists = true;
          }
          if(!exists)
            filenames.push_back(boost::get<hexabus::include_doc>(block).filename);
        }
      }
    } else {
      okay = false;
      if(!r)
        std::cout << "Parsing of file " << filenames[f] << " failed." << std::endl;
      if(r)
        std::cout << "Parsing of file " << filenames[f] << " failed: Did not reach end of file." << std::endl;
    }
  }

  if(okay) {
    if(vm.count("print")) {
      hexabus::hbc_printer printer;
      printer(ast);
    } else if(vm.count("graph")) {
      hexabus::GraphBuilder gBuilder;
      gBuilder(ast);
      std::ofstream ofs;
      std::string outfile(vm["graph"].as<std::string>());
      if(std::string("") == outfile) {
        std::cout << "No graph output file specified." << std::endl;
        exit(-1);
      }
      ofs.open(outfile.c_str());
      if(!ofs) {
        std::cerr << "Error: Could not open graph output file." << std::endl;
        exit(-1);
      }
      gBuilder.write_graphviz(ofs);
      ofs.close();
    } else if(vm.count("tables")) {
      hexabus::TableBuilder tableBuilder;
      tableBuilder(ast);
      tableBuilder.print();
    }
  } else {
    return 1;
  }

  return 0;
}