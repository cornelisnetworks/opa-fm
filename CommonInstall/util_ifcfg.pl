#!/usr/bin/perl
## BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# END_ICS_COPYRIGHT8   ****************************************

# [ICS VERSION STRING: unknown]

#

use strict;
#use Term::ANSIColor;
#use Term::ANSIColor qw(:constants);
#use File::Basename;
#use Math::BigInt;

# ==========================================================================
# IP address configuration
# used for IPoIB at present but generalized so could support VNIC or
# iPathEther also

my $FirstIPoIBInterface=0; # first device is ib0
my $MAX_HFI_PORTS=20;	# maximum valid ports

# Validate the passed in IP address (or netmask).
# Verify there are enough dots '.' and same number of
# decimal numbers. This is not comprehensive.
#
# inputs:
#	IP address in dot notation.  eg:
#	'a.b.c.d' or 'a.b.c' or 'a.b'
#
# outputs:
#	0 == failure(not a valid IP address)
#	1 == Success.

sub Validate_IP_Addr($)
{
	$_ = shift();		# setup for translate & other cmds

	my($count)= 0;

	# verify there are 3 and only 3 dots, 'a.b.c.d'.
	# translate '.' to self, side-effect is the count.

	$count=(tr/\.//);
	if ( $count < 1 || $count > 3 )
	{
		return 0;
	}

	# accept only decimal digits separated by dots.
	if ( /[0-9\.]/ ) 
	{
		# because we checked dots above, any of the three below is ok
		if ( /([0-9]+)\.([0-9]+)\.([0-9]+)\.([0-9]+)/ ) 
		{
			return 1;	# Success
		}
		if ( /([0-9]+)\.([0-9]+)\.([0-9]+)/ ) 
		{
			return 1;	# Success
		}
		if ( /([0-9]+)\.([0-9]+)/ ) 
		{
			return 1;	# Success
		}
	}
	return 0;	# failure.
}


# Retrieve an IPV4 address in standard 'dot' notation from stdin.
# Perform simple IP address format validation.
#
# returns:
#	IP address string.
#

sub Get_IP_Addr($$)
{
	my($interface) = shift();
	my($default_ip) = shift();

	my($ip,$idx);

	# acquire a valid IP address.
	# sadly last & redo caused runtime errors?

	do 
	{
		if ("$default_ip" ne "")
		{
			print "Enter IPV4 address in dot notation (or dhcp) for $interface [$default_ip]: ";
		} else {
			print "Enter IPV4 address in dot notation (or dhcp) for $interface: ";
		}
		chomp($ip = <STDIN>);
		$ip=remove_whitespace($ip);

		if ( length($ip) == 0 && "$default_ip" ne "" ) 
		{
			$ip = $default_ip;
			$_ = "y";
		}
		else 
		{
			# verify there are enough chars for a valid IP address
			if ( "$ip" ne "dhcp" && Validate_IP_Addr($ip) == 0 ) 
			{
				print("Bad IPV4 address '$ip'\n");
				$_ = "n";
			}
			else 
			{
				do
				{
					# require a response
					print("Is IPV4 address '$ip' correct? (y/n): ");
					chomp($_ = <STDIN>);
					$_=remove_whitespace($_);
				} until (/[YyNn]/);
			}
		}
	} until (/[Yy]/);
	LogPrint "Enter IPV4 address in dot notation for $interface: -> $ip\n";

	return $ip;
}

# Retrieve an IPV4 netmask in standard 'dot' notation from stdin.
# Perform simple IP address format validation.
#
# returns:
#	IP address string.
#

sub Get_IP_Addr_Netmask($$)
{
	my($interface) = shift();
	my($ipaddr) = shift();

	my($idx,$default_netmask,$dot_count);
	my $netmask;

	if ( "$ipaddr" eq "dhcp" ) {
		return "";
	}
	$default_netmask = `/usr/lib/opa/tools/opaipcalc --netmask $ipaddr`;
	$default_netmask =~ s/NETMASK=//;
	$default_netmask =~ s/\n//;
	$_ = $ipaddr;		# setup for translate & other cmds
	$dot_count=(tr/\.//);
	do 
	{
		print "Enter IPV4 netmask in dot notation for $interface $ipaddr [$default_netmask]: ";
		chomp($netmask = <STDIN>);
		$netmask=remove_whitespace($netmask);
		$_=$netmask;

		if ( length($netmask) == 0 ) 
		{
			$netmask = $default_netmask;
			print("Is IPV4 netmask '$netmask' correct? (y/n): ");
			chomp($_ = <STDIN>);
			$_=remove_whitespace($_);
		}
		else 
		{
			# verify there are enough chars for a valid IP address
			if ( Validate_IP_Addr($netmask) == 0 ) 
			{
				print("Bad IPV4 netmask: '$netmask'\n");
				$_ = "n";
			}
			elsif ($dot_count != (tr/\.//))
			{
				print("Inappropriate IPV4 netmask for $ipaddr: '$netmask'\n");
				$_ = "n";
			}
			else 
			{
				print("Is IPV4 netmask '$netmask' correct? (y/n): ");
				chomp($_ = <STDIN>);
				$_=remove_whitespace($_);
			}
		}
	} until (/[Yy]/);
	LogPrint "Enter IPV4 netmask in dot notation for $interface $ipaddr [$default_netmask]: -> $netmask\n";
	return $netmask;
}

# return the next sequential IPV4 address. Last numeric field is incremented.
#
# Assumes a valid dot notation IP address (e.g., 7.7.7.240)
# Increment the 'host' number ('240' in the example) by 1.
#  7.7.7.240 --> 7.7.7.241
#
# input:
#	base IP address in 'dot' notation.
# output:
#	next IP address

sub NextIPaddr($)
{
	my($base) = shift();

	my($next,$hostnum,$idx,$len);

	if ( "$base" eq "dhcp" ) {
		return "$base";
	}
	# locate right-most '.'
	$idx = rindex($base, ".");
	Abort "NextIPaddr: Bad IPV4 address ".$base
		if ($idx == -1);

	$idx += 1;	# move past the dot to the last numeric field portion of
				# the IP address.

	# extract the host number
	$len = length($base);
	$hostnum = substr($base, ($idx), ($len - $idx));

	$hostnum += 1;	# next host number

	# replace the last numeric field with the incremented host number
	$next = $base;
	substr($next,$idx,length($hostnum)) = $hostnum;

	DebugPrint("Next IP '$next'\n");

	return $next
}

# return the next sequential interface name. Last numeric field is incremented.
# if the final field is not numeric, returns ""
sub NextInterface($)
{
	my($base) = shift();

	my($next,$hostnum,$idx,$len);
	my $number;
	my $prefix;

	$_ = $base;
	if ( ! /.*([0-9]+)$/ ) 
	{
		# non numeric name, can't auto-increment
		return "";
	}
	$prefix = $base;
	$prefix =~ s/(.*)([0-9]+)$/$1/;
	$number = $base;
	$number =~ s/(.*)([0-9]+)$/$2/;

	$number += 1;	# next intf number

	return "$prefix$number";
}

# build the ifcfg file for a network device
sub Build_ifcfg($$$)
{
	my($device) = shift();		# device name
	my($ipaddr) = shift();		# ip address for device or "dhcp"
	my($netmask) = shift();		# ip address netmask for device

	my($target, $SysCmd);
	my ($temp);

	$target = "$NETWORK_CONF_DIR/ifcfg-$device";
	print("Creating ifcfg-$device for $ipaddr mask $netmask\n\n");

	if ( "$CUR_DISTRO_VENDOR" eq "UnitedLinux" || "$CUR_DISTRO_VENDOR" eq "SuSE") {
		if ( "$ipaddr" eq "dhcp" ) {
			# append boot protocol type
			$SysCmd = "echo \"BOOTPROTO=\'dhcp\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;
		} else {
			# append boot protocol type
			$SysCmd = "echo \"BOOTPROTO=\'static\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# append the device instance internet protocol address
			$SysCmd = "echo \"IPADDR=\'$ipaddr\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# append the netmask
			$SysCmd = "echo \"NETMASK=\'$netmask\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# append the network
			$temp = `/usr/lib/opa/tools/opaipcalc --network $ipaddr $netmask`;
			chomp($temp);
			$temp =~ s/NETWORK=//;
			$SysCmd = "echo \"NETWORK=\'$temp\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# append the broadcast
			$temp = `/usr/lib/opa/tools/opaipcalc --broadcast $ipaddr $netmask`;
			chomp($temp);
			$temp =~ s/BROADCAST=//;
			$SysCmd = "echo \"BROADCAST=\'$temp\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			$SysCmd = "echo \"REMOTE_IPADDR=\'\'\" >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;
		}

		$SysCmd = "echo \"STARTMODE=\'hotplug\'\" >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;

		$SysCmd = "echo \"WIRELESS='no'\" >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;

		# SLES11 and newer have IPOIB_MODE option in ifcfg
		$SysCmd = "echo \"IPOIB_MODE='connected'\" >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;
		$SysCmd = "echo \"MTU=65520\" >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;
	} else {
		# Append the device instance name
		$SysCmd = "echo DEVICE=$device > $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;

		$SysCmd = "echo \"TYPE=\'InfiniBand\'\" >> $target";
                DebugPrint("cmd '$SysCmd'\n");
                system $SysCmd;

		if ( "$ipaddr" eq "dhcp" ) {
			$SysCmd = "echo BOOTPROTO=dhcp >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			$SysCmd = "echo DHCPCLASS= >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;
		} else {

			$SysCmd = "echo BOOTPROTO=static >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# Append the device instance Internet Protocol address
			$SysCmd = "echo IPADDR=$ipaddr >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# append the network,broadcast
			$SysCmd = "/usr/lib/opa/tools/opaipcalc --network --broadcast $ipaddr $netmask >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;

			# append the netmask
			$SysCmd = "echo NETMASK=$netmask >> $target";
			DebugPrint("cmd '$SysCmd'\n");
			system $SysCmd;
		}

		$SysCmd = "echo ONBOOT=yes >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;

		$SysCmd = "echo CONNECTED_MODE=yes >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;
		$SysCmd = "echo MTU=65520 >> $target";
		DebugPrint("cmd '$SysCmd'\n");
		system $SysCmd;
	}

	system "chown $OWNER $target";
	system "chgrp $GROUP $target";
	system "chmod ugo=r,u=rw $target";
}

sub Config_IP_Manually($$)
{
	my($compname) = shift();
	my($sampledev) = shift();

	print "\n$compname requires an ifcfg file for each $compname device instance.\n";
	print ("Manually create files such as '$NETWORK_CONF_DIR/ifcfg-$sampledev'\n");
}


sub Exist_ifcfg($)
{
	my($prefix) = shift();

	my $ifcfg_wildcard="$NETWORK_CONF_DIR/ifcfg-$prefix"."[0-9]*";
	return ( `ls $ifcfg_wildcard 2>/dev/null` ne "" );
}

#
# Generate the network interface config files for IP over IB
#	/etc/sysconfig/network-scripts/ifcfg-ibl1
#
# Assign IP (Internet Protocol) addresses in standard 'dot' notation to
# IP over IB device instances. Create IP over IB 'ifup ibX' configuration files
# located in '/etc/sysconfig/network-scripts' for each IP over IB port.
#
# **** Currently - ONLY static IP address assignment is supported.
#
# inputs:
#	[0] == are we reconfiguring, if 1, allows editing of existing ifcfg files
#
# outputs:
#	none.
#
sub Config_ifcfg($$$$$)
{
	my($reconfig) = shift();
	my($compname) = shift();
	my($prefix) = shift();
	my($firstdev) = shift();
	my($showoptions) = shift();

	my($max_ports,$port,$a,$inp,$seq,$intf_seq,$default_intf);
	my($netmask);
	my($ipaddr) = "";
	my($interface) = "";
	my(%interfaces) = ();
	my $config_dir;

	$max_ports = 0;
			
	# Check if ifcfg files are present
	my $ifcfg_wildcard="$NETWORK_CONF_DIR/ifcfg-$prefix"."[0-9]*";
	if ( `ls $ifcfg_wildcard 2>/dev/null` ne "" )
	{
		if ($reconfig)
		{
			# always prompt regardless of previous check_keep_config answer
			clear_keep_config("$ifcfg_wildcard");
		}
		if (! $reconfig 
			|| check_keep_config("$ifcfg_wildcard", "$compname ifcfg files", "y"))
		{
			print "Leaving $ifcfg_wildcard unchanged...\n";
			return;
		} 
		print "removing $ifcfg_wildcard\n";
		system "rm -f $ifcfg_wildcard";
	}
	if (GetYesNo("Configure $compname IPV4 addresses now?", "n") == 0)
	{
		# If user answered 'no', then reluctantly bail.
		Config_IP_Manually("$compname","$prefix$firstdev");
		return;
	}

	if ( $showoptions != 0 )
	{
		printf ("\nYou may configure an $compname interface for each HFI port\n");
		printf ("Or you may select to have $compname only run on some HFI ports\n");
		printf ("Or you may select to configure redundant HFI ports with a\n");
		printf ("pair of HFI ports running a single $compname interface\n");
	}
	do
	{
		printf ("How many $compname interfaces would you like to configure? [1]: ");

		$inp = <STDIN>;
		$inp=remove_whitespace($inp);

		if ( length($inp) == 0 ) 
		{
			$max_ports=1;
		} elsif ($inp < 0 || $inp > $MAX_HFI_PORTS)
		{
			printf ("You must specify a number between 0 and $MAX_HFI_PORTS\n");
		} elsif ($inp == 0) {
			LogPrint "How many $compname interfaces would you like to configure? -> $inp\n";
			if (GetYesNo("You specified 0, would you like to skip IP address configuration?", "n") == 1)
			{
				# If user answered to skip, then reluctantly bail.
				Config_IP_Manually("$compname","$prefix$firstdev");
				return;
			}
		} else {
			$max_ports=$inp;
			LogPrint "How many $compname interfaces would you like to configure? -> $inp\n";
		}
	} while ($max_ports == 0);

	print "\nAbout to create $compname ifcfg files in $NETWORK_CONF_DIR\n";

	# Does the user want sequential IP address assignment starting at a base
	# IP address or specify an IP address for each HFI port (a.k.a. IPoIB device
	# instance). 

	if ( $max_ports > 1 ) {
		$intf_seq = GetYesNo("Configure interface names sequentially starting with $prefix$firstdev?", "y");
		$seq = GetYesNo("Assign Internet Addresses sequentially from a base IP address?", "y");
	} else {
		$intf_seq = GetYesNo("Use interface name $prefix$firstdev?", "y");
		$seq = 1;	# doesn't matter
	}

	# Allocate IP addresses and create the network config files '$prefix[firstdev...n]'.

	for($a=$firstdev; $a < $max_ports+$firstdev; $a++)
	{
		if ($intf_seq)
		{
			$interface="$prefix$a";
		} else {
			$default_intf = $interface;
			$interface = "";
			do {
				if ($default_intf eq "" )
				{
					print "Enter $compname interface name: ";
				} else {
					print "Enter $compname interface name [$default_intf]: ";
				}

				chomp($inp = <STDIN>);
				$inp=remove_whitespace($inp);
				if ( length($inp) == 0 )
				{
					$inp=$default_intf;
				}
				if ("$inp" eq "" )
				{
					printf "You must enter an interface name, such as $prefix$firstdev\n";
				} else {
					if (exists $interfaces{$inp})
					{
						print "You have already used $interface, specify a different name\n";
					} else {
						$interface=$inp;
						$interfaces{$interface} = 1;
					}
				}
			} while ("$interface" eq "");
		}
		LogPrint "Enter $compname interface name [$default_intf]: -> $interface\n";
		if ( $seq == 0 || $a == $firstdev) 
		{
			# not sequential assignment, get each IP addr.
			$ipaddr = Get_IP_Addr($interface, $ipaddr);
			$netmask = Get_IP_Addr_Netmask($interface, $ipaddr);
		}

		Build_ifcfg($interface, $ipaddr, $netmask);
		# make sure future prompts start fresh and don't auto remove
		clear_keep_config("$ifcfg_wildcard");

		# generate the next sequential IP address
		$ipaddr = NextIPaddr($ipaddr);
		$interface = NextInterface($interface);
	}
}

# Remove ifcfg files.
# driver should have already been unloaded.
#
sub Remove_ifcfg($$$)
{
	my($WhichDriver) = shift();	# driver name
	my($compname) = shift();
	my($prefix) = shift();

	# This could be an 'update' where we might not want to clobber the
	# IPoIB config files.

	my $ifcfg_wildcard="$NETWORK_CONF_DIR/ifcfg-$prefix"."[0-9]*";
	if ( `ls $ifcfg_wildcard 2>/dev/null` ne "" )
	{
		if (check_keep_config("$ifcfg_wildcard", "$compname ifcfg files", "y"))
		{
			print "Keeping $compname ifcfg files ...\n";
		} else {
			print "removing $ifcfg_wildcard\n";
			system "rm -f $ifcfg_wildcard";
		}
	}
}

sub check_network_config()
{
	my $nm=read_simple_config_param("/etc/sysconfig/network/config", "NETWORKMANAGER");
	if ( "$nm" eq "yes" ) {
		print RED "Please set NETWORKMANAGER=no in /etc/sysconfig/network/config", RESET "\n";
		HitKeyCont;
	}
}
