#!/usr/bin/env python3

import ipaddress
import subprocess
import io
import re
import random
import argparse

class DiscoveryFailed(Exception):
	"""Discovery of networks failed"""

def discover_present_prefixes(iface):
	"""List all prefixes seen by the machine.

	Calls rdisc6 to aquire a list a prefixes and routes announced on the network.

	Args:
		iface: name of the interface to ask for routes

	Returns:
		A tuple (prefixes, routes) of IPv6 networks reachable by this node
	"""
	prefixes = []
	routes = []
	with subprocess.Popen(["rdisc6", iface], stdout=subprocess.PIPE) as proc:
		tio = io.TextIOWrapper(proc.stdout)
		line = tio.readline()
		while len(line) > 0:
			parts = re.match("^ (Prefix|Route)\s*:\s*([^\s]*)", line)
			if parts != None:
				if parts.group(1) == "Prefix":
					prefixes.append(ipaddress.ip_network(parts.group(2)))
				else:
					routes.append(ipaddress.ip_network(parts.group(2)))
			line = tio.readline()
			proc.wait()
			if proc.returncode != 0 and proc.returncode != 2:
				raise DiscoveryFailed
	return (prefixes, routes)

def net_contains(net, cand):
	"""Checks whether a network fully contains another network

	Args:
		net: candidate supernet
		cand: candidate subnet

	Returns:
		True if and only if net is a strict supernet of cand
	"""
	return cand.network_address in net and cand.broadcast_address in net

def exclude_all(netlist):
	"""Calculates a list of private prefixes that are not subnets of given networks

	Args:
		netlist: a list of networks

	Returns:
		A list of networks in fd00::/8 that are not subnets of nets in netlist
	"""
	result = [ ipaddress.ip_network("fd00::/8") ]
	for i in netlist:
		next = []
		for net in result:
			if net_contains(net, i):
				next.extend(net.address_exclude(i))
			elif net_contains(i, net):
				next.extend(i.address_exclude(net))
		result = next
	return result

def select_networks(prefixes, unusedPrefixes):
	"""Select a suitable pair of prefixes for a HxB router node

	Args:
		prefixes: list of prefixes available for SLAAC
		unusedPrefixes: list if viable prefixes for the HxB network

	Returns:
		A tuple (eth, hxb) of prefixes for the ethernet side and the hexabus side
		of the network. eth may be None if an IPv6 network exists already and SLAAC
		should be used for the ethernet side of things
	"""
	hxb = random.choice(unusedPrefixes)
	while hxb.prefixlen < 63:
		hxb = random.choice(list(hxb.subnets(1)))
	if len(prefixes) > 0:
		return (None, list(hxb.subnets(1))[1])
	else:
		nets = list(hxb.subnets(1))
		return (nets[0], nets[1])

def generate_interfaces_fragment(prefix, interface):
	"""Generates a part of /etc/network/interfaces

	Args:
		prefix: prefix to use for the interface
		interface: interface to configure

	Returns:
		Configuration fragment for /etc/network/interfaces
	"""
	header = """auto {0}
allow-hotplug {0}

"""
	slaac = """iface {0} inet6 auto
	privext 1
	dhcp 0
"""
	static = """iface {0} inet6 static
	address {1}
	netmask {2}
	privext 0
	scope site
"""
	if prefix is None:
		return (header + slaac).format(interface)
	else:
		return (header + static).format(interface, next(prefix.hosts()), prefix.prefixlen)

def generate_radvd_fragment(prefix, interface, routes = None):
	"""Generate a radvd.conf fragment to announce a given prefix, possibly with routes

	Args:
		prefix: prefix to announce for SLAAC. If None, SLAAC will be disabled
		routes: routes to announce. Include ::/0 to enable default routes

	Returns:
		A configuration fragment for a radvd.conf
	"""
	header = """interface {0} {{
	IgnoreIfMissing on;
	AdvSendAdvert on;

""".format(interface)
	prefix_fragment = ""
	if prefix is not None:
		prefix_fragment = "\tprefix {0} {{ }};\n".format(prefix)
	footer = "};"
	routes_fragment = ""
	if routes is not None:
		for route in routes:
			routes_fragment += "\troute {0} {{ }};\n".format(route)
	return header + prefix_fragment + routes_fragment + footer

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description = "Autoconfiguration utility for Hexabus routers")
	parser.add_argument("eth", type = str, help = "Ethernet interface name")
	parser.add_argument("hxb", type = str, help = "Hexabus interface name")

	try:
		args = parser.parse_args()

		(prefixes, routes) = discover_present_prefixes(args.eth)
		viable = exclude_all(prefixes + routes)
		nets = select_networks(prefixes, viable)

		eth_iface = generate_interfaces_fragment(nets[0], args.eth)
		eth_radvd = ""
		if nets[0] is not None:
			eth_radvd = generate_radvd_fragment(nets[0], args.eth, [nets[1]])

		hxb_iface = generate_interfaces_fragment(nets[1], args.hxb)
		hxb_radvd = generate_radvd_fragment(nets[1], args.hxb, [ipaddress.IPv6Network("::/0")])

		print("====== eth ======")
		print(eth_iface);
		print(eth_radvd)

		print("====== hxb ======")
		print(hxb_iface);
		print(hxb_radvd)
	except DiscoveryFailed:
		exit(1)
	except:
		exit(2)
