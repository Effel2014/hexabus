#include "graph_transformation.hpp"

#include <libhbc/error.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <libhbc/hbc_printer.hpp>
#include <boost/foreach.hpp>

using namespace hexabus;

machine_map_t GraphTransformation::generateMachineMap(graph_t_ptr g) {
  machine_map_t machine_map; // TODO maybe we want to have a graph_t instead of a vector<vertex_id_t>?? ... or probably we want to construct the graph later. Handling should be easier that way.
  // iterate over all vertices in graph
  graph_t::vertex_iterator vertexIt, vertexEnd;
  boost::tie(vertexIt, vertexEnd) = vertices(*g);
  for(; vertexIt != vertexEnd; vertexIt++) {
    vertex_id_t vertexID = *vertexIt;
    // look for machine ID in map
    machine_map_t::iterator node_it;
    if((node_it = machine_map.find((*g)[vertexID].machine_id)) != machine_map.end()) {
      // if there, append found vertex to vector in map
      node_it->second.push_back(vertexID);
    } else {
      std::vector<vertex_id_t> vertex_vect;
      vertex_vect.push_back(vertexID);
      // if not there, make new entry in map
      machine_map.insert(std::pair<unsigned int, std::vector<vertex_id_t> >((*g)[vertexID].machine_id, vertex_vect));
    }
  }
  // okay. now we have a map mapping from each state machine ID to the vertices belonging to this state machine. (1)

  return machine_map;
}

std::vector<std::string> GraphTransformation::findDevices(std::vector<vertex_id_t> stm_vertices, graph_t_ptr g) {
  std::vector<std::string> devices;

  // iterate over all vertices in the state machine
  BOOST_FOREACH(vertex_id_t v_id, stm_vertices) {
    if((*g)[v_id].type == v_command) { // only look at command block vertices
      std::string device_name = "";
      try {
        command_block_doc cmdblck = boost::get<command_block_doc>((*g)[v_id].contents);
        // iterate over all commands in the command block
        BOOST_FOREACH(command_doc cmd, cmdblck.commands) {
          std::string device_name = boost::get<std::string>(cmd.write_command.geid.device_alias);
          if(!exists(device_name, devices))
            devices.push_back(device_name);
        }
      } catch(boost::bad_get b) {
        throw GraphTransformationErrorException("Contents of command vertex are not a command block");
      }
    }
  }

  return devices;
}

template <typename T> bool GraphTransformation::exists(T elem, std::vector<T> vect) {
  bool exists = false;
  BOOST_FOREACH(T the_elem, vect)
    if(the_elem == elem)
      exists = true;
  return exists;
}

void GraphTransformation::operator()(graph_t_ptr in_g) {
  // TODO:                                                | Data structures needed:
  // * iterate over (list of) state machines              |
  // * for each state machine:                            |
  //   * make list of devices  (1)                        |
  //   * slice each state machine for each device         | * list of state machine slices for devices
  // * if a device appears in multiple state machines:    |   (at this point one dev. can have several state machines)
  //   -> parallel compose state machine slices           | * now, each dev. only has one st.m. (store as map?)
  // * simplify slices for Hexabus Assembler export       | * use the same map data structure for this

  machine_map_t machines_per_stmid = generateMachineMap(in_g); // list of nodes for each state machine ID

  BOOST_FOREACH(machine_map_elem mm_el, machines_per_stmid) {
    std::vector<std::string> device_names = findDevices(mm_el.second, in_g); // list of devices for this state machine

    // not we have a list off all device names used in state machine #mm_el.first
  }
}
