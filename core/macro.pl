#!/usr/bin/perl
#
# Copyright (c) 2013, Peter Rutenbar <pruten@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

use strict;
use Carp;
use Storable qw(dclone);

my $tab = "    "; # one tab => four spaces

main();

sub main {
	
	my $input = "";
	
	if (($#ARGV) != 1) {
		my $args = $#ARGV + 1;
		croak "I need two arguments (got $args)";
	}

	open(INPUT, $ARGV[0]) or die croak "I can't open $ARGV[0]!";
	while (my $line = <INPUT>) {
		$input .= $line;
	}
	close(INPUT);
	
	$input = tabs_to_spaces($input); # it's simpler to deal with if all indentation can be represented as a # of spaces
	my $ctx = {
		text => line_logic($input), # the input text
		out => "",                  # the output text
		depth => 0,                 # the recursive depth of the parser
        filename => $ARGV[0],       # the name of the input file
		adhoc => {}                 # hash of adhoc function refs
	};
	parse($ctx);

	open(OUTPUT, '>'.$ARGV[1]);
	print OUTPUT "/* Generated from $ARGV[0] */\n\n";
	print OUTPUT $ctx->{out};
	close(OUTPUT);
}

sub tabs_to_spaces {
	my @chars = split //,shift;
	my $output = "";
	foreach my $c (@chars) {
		if ($c eq "\t") {
			$output .= $tab; # a tab is 4 spaces (imho)
		} else {
			$output .= $c;
		}
	}
	return $output;
}

sub line_logic {
	# For simplicity, line numbers start at 0
	my $raw = shift;
	my @lines = split /\n/,$raw;
	
	my @indents; # $indents[n] => the indent prefix for nth line
	my @chars; # all the characters in the input text
	my @line_map; # $line_map[n] => the line number to which the nth character belongs

	my $line_no = 0;
	foreach my $line (@lines) {
		my $past_indent = 0;
		my $indent_count = 0;
		my @l_chars = split //, $line; 	
		foreach my $c (@l_chars) {
			unless ($c eq " ") {
				$past_indent = 1;
			}
			if ($past_indent) {
				push @chars, $c;
				push @line_map, $line_no;
			}
			else {
				$indent_count++;
			}
		}
		push @indents, $indent_count;
		push @chars, "\n";
		push @line_map, $line_no;
		$line_no++;
	}

	return {
		indents => \@indents,
		chars => \@chars,
		line_map => \@line_map,
		line_no => $line_no
	};
}

sub spaces {
	my $count = shift;
	my $out = "";
	for (my $i=0; $i < $count; $i++) {
		$out .= " ";
	}
	return $out;
}

sub parse {
	my $ctx = shift;
	my $out = "";
	my $text = $ctx->{text};

	my $newline = 1;
	for (my $i=0; $i < scalar(@{$text->{chars}}); $i++) { # iterate over every character in the input
		my $c = $text->{chars}->[$i];
		my $line_no = $text->{line_map}->[$i];
		$ctx->{current_line} = $line_no+1; # this is so macros can know what line they were called from

		if ($newline) { # a newline just began
			$newline = 0;
			$ctx->{indent} = $text->{indents}->[$line_no]; # keep track of how many indents are on this line
			$out .= spaces($ctx->{indent}); # and begin the line with the appropriate indentation
		}

		if ($c ne "~") {
			$out .= $c;
		} else {
			my $line_str = "";
			$ctx->{cur_pos} = $i;
			
			my $macro = resolve_macro($ctx);
			
			$i = $ctx->{cur_pos};
			$out .= join("\n$line_str".spaces($ctx->{indent}), split(/\n/, $macro->{str})); 
		}

		if ($c eq "\n") { # a newline begins on the next character
			$newline = 1;
		}
	}

	$ctx->{out} = $out;
}

sub trim {
	my $str = shift;
	$str =~ s/^\s+//;
	$str =~ s/\s+$//;
	return $str;
}

sub resolve_macro {
	my $ctx = shift;
	my $text = $ctx->{text};
	my $chars = $text->{chars};
	my $i = $ctx->{cur_pos};
	
	# "~~" => "~"
	if (($chars->[$i] eq "~") and ($chars->[$i+1] eq "~")) {
		$ctx->{cur_pos} = $i+1;
		return {str => "~"};
	}
	
	# ~macro_name(arg1, arg2, {
	# 	printf("code goes here");
	# })
	
	# parse out the macro name
	my $macro_name = "";
	for ($i++; ($i < scalar(@$chars)) and ($chars->[$i] ne "("); $i++) {
		$macro_name .= $chars->[$i];
	}
	if ((length($macro_name) > 80) or ($i == scalar(@$chars)) or ($chars->[$i] ne "(")) {
		croak sprintf("line=%u: I call bullshit on this macro call: starts \"%s...\"",
			1+$text->{line_map}->[$ctx->{cur_pos}], substr($macro_name, 0, 10));
	}
	$macro_name = trim($macro_name);
	
	# parse out the macro arguments
	my @macro_args;
	for ($i++; ($i < scalar(@$chars)) and ($chars->[$i] ne ")"); ) {
	
		if (($chars->[$i] =~ /\s/) or ($chars->[$i] eq ',')) {
			$i++;
		}
		elsif ($chars->[$i] eq "{") {
			# process code block
			$ctx->{cur_pos} = $i;
			my $block = do_parse_recurse($ctx);
			$i = $ctx->{cur_pos};
			push @macro_args, $block;
		}
		else {
			# plain text argument
			my $arg = "";
			my $paren_count = 1;
			for (; ($i < scalar(@$chars)) and (!(($paren_count==1)and($chars->[$i]eq')'))) and ($chars->[$i] ne ","); $i++) {
				if ($chars->[$i] eq ')') {$paren_count--;}
				elsif ($chars->[$i] eq '(') {$paren_count++;}
				$arg .= $chars->[$i];
				# printf("character = %s, paren_count = %u\n", $chars->[$i], $paren_count);
			}

			if ($i == scalar(@$chars)) {
				croak(sprintf('line=%u: argument #%u seems bogus', $text->{line_map}->[$ctx->{cur_pos}], scalar(@macro_args)));
			}
			$arg = trim($arg);
			
			# if the argument is encased in quotes, eval it
			if ( (length($arg)>0) and ( (substr($arg, 0, 1) eq "'") or (substr($arg, 0, 1) eq "\"") ) ) {
				my $newarg = "";
				eval '$newarg'." = $arg;";
				croak(sprintf("line=%u: eval() doesn't like argument at all: \"%s\" (%s)", 
					1+$text->{line_map}->[$ctx->{cur_pos}], $arg, $@)) if $@;
				$arg = $newarg;
			}

			push @macro_args, $arg;
		}
	}
	
	# evaluate macro
	my $macro_call;
	my $macro_return;
	if (exists $ctx->{adhoc}->{$macro_name}) {
		$macro_call = sprintf('$macro_return = $ctx->{adhoc}->{%s}->(\@macro_args,$ctx);', $macro_name);
	} else {
		$macro_call = sprintf('$macro_return = macro_%s(\@macro_args,$ctx);', $macro_name);
	}
	eval $macro_call;
	if ($@) {
		croak(sprintf("line=%u: Macro failed (exploded):\n%s", 1+$text->{line_map}->[$ctx->{cur_pos}], $@));
	}
	
	$ctx->{cur_pos} = $i;
	return {str => $macro_return};
}

sub do_parse_recurse {
	my $oldctx = shift;
	my $oldtext = $oldctx->{text};
	my $oldchars = $oldtext->{chars};
	my $oldline_map = $oldtext->{line_map};
	my $old_indents = $oldtext->{indents};
	
		# dclone can't handle CODE references, apparently
		my $adhoc_backup = $oldctx->{adhoc};
		delete $oldctx->{adhoc};
	my $newctx = dclone($oldctx);
		$oldctx->{adhoc} = $adhoc_backup;
		$newctx->{adhoc} = {};
		foreach my $key (keys %$adhoc_backup) {
			$newctx->{adhoc}->{$key} = $adhoc_backup->{$key};
		}
	
	my $blockstart = $oldctx->{cur_pos}; # points to '{'
	my $i = $blockstart+1; # the first character we're given is '{', skip past it
	for (my $curly_level=1; ($i < scalar(@$oldchars)) and ($curly_level != 0); $i++) {
		if ($oldchars->[$i] eq '{') {$curly_level++;}
		elsif ($oldchars->[$i] eq '}') {$curly_level--;}
	}
	my $blockend = $i - 1; # points to '}'
	my $blocklen = $blockend - ($blockstart + 1); # the length of the text inside the block
	
	my @newchars = @{$oldchars}[($blockstart+1)..($blockend-1)];
	my @newline_map = @{$oldline_map}[($blockstart+1)..($blockend-1)];
	
	my $orig_indent = $old_indents->[$oldline_map->[$blockstart]];
	for (my $i=0; $i < $blocklen; $i++) {
		my $oldi = $blockstart+1+$i;
		my $line = $newline_map[$i];
		
		if ($old_indents->[$line] >= $orig_indent) {
			$newctx->{text}->{indents}->[$line] = $old_indents->[$line] - $orig_indent;
		} else {
			$newctx->{text}->{indents}->[$line] = 0;
		}
	}
	
	$newctx->{text}->{chars} = \@newchars;
	$newctx->{text}->{line_map} = \@newline_map;
	$newctx->{depth}++;
	$newctx->{out} = "";
	
	parse($newctx);
	
	$oldctx->{cur_pos} = $blockend+1;
	return $newctx->{out};
}

sub count_args {
	my ($name, $args, $ctx, $num) = @_;
    if ($num == -1) {return ;}
	if ((scalar(@$args)) != $num) {
		croak(sprintf("line %u: ~$name: I expect $num arguments, but I got %u instead.", $ctx->{current_line}, scalar(@$args)));
	}
}


# ------------------------------------------------------------------
# Macros go here:
# ------------------------------------------------------------------

# ~decompose(op, "0101 ab0cd mmmrrr")
sub macro_decompose {
	my ($args, $ctx) = @_;
	count_args("decompose", $args, $ctx, 2);
	
	my ($op, $input_pattern) = @$args;
	my @raw = split //, $input_pattern;
	my @spaceless;
	foreach my $c (reverse(@raw)) {
		if ($c ne " ") {push @spaceless, $c;}
	}
	
	if (scalar(@spaceless) != 16) {
		carp sprintf("line %u: ~decompose: the pattern doesn't have 16 bits (or something else is wrong with it)", $ctx->{current_line});
	}
	
	my $lastc = "";
	my %symbols;
	for (my $i = 0; $i < scalar(@spaceless); $i++) {
		my $c = $spaceless[$i];
		if (index("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ", $c) != -1) {
			if (exists $symbols{$c}) {
				if ($lastc ne $c) {
					croak sprintf("line %u: ~decompose: the pattern uses '%s' more than once", $ctx->{current_line}, $c);
				}
				$symbols{$c}->{len}++;
			}
			else {
				$symbols{$c} = {	
					start => $i,
					len => 1
				};
			}
		}
		
		$lastc = $c;
	}
	
	my $output = "";
	foreach my $key (keys %symbols) {
		my $dat = $symbols{$key};
		my $mask = sprintf("0x%x", (2 ** $dat->{len})-1);
		$output .= sprintf("const uint16_t %s = ((%s)>>%u)&%s;\n", $key, $op, $dat->{start}, $mask);
	}
	
	return $output;
}

# ~b(1010) == 5
sub macro_b {
	my $args = shift;
	my $ctx = shift;
	if (scalar(@$args) != 1) {
		croak(sprintf("line %u: ~bin: ~bin() expects 1 argument, got %u", $ctx->{current_line}, scalar(@$args)));
	}
	my @digits;
	foreach my $c (split //, $args->[0]) {
        unless ($c eq ' ') {
            push @digits, $c;
        }
    }
	my $count = 0;
	my $val = 0;
	foreach my $c (@digits) {
		if ($c eq "0") {
			$val = ($val * 2);
		} elsif ($c eq "1") {
			$val = ($val * 2) + 1;
		} else {
			croak(sprintf("line %u: ~bin: non-digit in [%s]", $ctx->{current_line}, $args->[0]));
		}
	}
	return sprintf("0x%x", $val);
}

# if (~bmatch(op, 1000 xxxx 01 xxx xxx)) {...}
sub macro_bmatch {
	my ($args, $ctx) = @_;
	count_args("bmatch", $args, $ctx, 2);
	
	my $op = $args->[0];
	my $pattern_raw = $args->[1];
	
	my @pattern;
	foreach my $c (split '', $pattern_raw) {
		if (($c eq 'x') or ($c eq '1') or ($c eq '0')) {
			push @pattern, $c;
		} elsif ($c ne ' ') {
			croak(sprintf("line %u: ~bmatch: I hate to be picky! But there's a bogus character in this bit pattern '%s'.", $ctx->{current_line}, $c));
		}
	}
	if ((scalar(@pattern) != 8) and (scalar(@pattern) != 16) and (scalar(@pattern) != 32)) {
		croak(sprintf("line %u: ~bmatch: The number of bits in this pattern isn't in {8,16,32} (it's %u).", $ctx->{current_line}, scalar(@pattern)));
	}
	
	(my $count=0, my $mask_val=0, my $eq_val=0, my $mask="0x", my $eq="0x");
	foreach my $c (@pattern) {
		if ($c eq '1') {
			$mask_val = ($mask_val * 2) + 1;
			$eq_val = ($eq_val * 2) + 1;
		} elsif ($c eq '0') {
			$mask_val = ($mask_val * 2) + 1;
			$eq_val = ($eq_val * 2) + 0;
		} else { # $c eq 'x'
			$mask_val = ($mask_val * 2) + 0;
			$eq_val = ($eq_val * 2) + 0;
		}
		$count++;
		if ($count == 8) {
			$mask .= sprintf("%02x", $mask_val);
			$eq .= sprintf("%02x", $eq_val);
			$mask_val = 0;
			$eq_val = 0;
			$count = 0;
		}
	}
	return sprintf("(((%s)&%s)==%s)", $op, $mask, $eq);
}

# ~newmacro(memcpy, 3, {return "memcpy($args->[0], $args->[1], $args->[2])";}
sub macro_newmacro {
	my ($args, $ctx) = @_;
	count_args("newmacro", $args, $ctx, 3);

	my ($name, $numargs, $code) = @$args;
	my $sub_ref;
	eval sprintf('$sub_ref = sub {my ($args, $ctx) = @_;count_args("%s", $args, $ctx, %d);%s};', $name, $numargs, $code);
	if ($@) {
		croak(sprintf("line %u: ~newmacro: failed to create adhoc macro: {%s}", $ctx->{current_line}, $@));
	}
	
	if (exists $ctx->{adhoc}->{$name}) {
		carp(sprintf("line %u: ~newmacro: warning! redefining already-existing adhoc function (%s)", $ctx->{current_line}, $name));
	}
	$ctx->{adhoc}->{$name} = $sub_ref;
	
	return "";
}

sub macro_bytes {
    my ($args, $ctx) = @_;
	count_args("bytes", $args, $ctx, 1);
    my $input_str = $args->[0];
    
    my %tab;
    foreach my $c (qw(0 1 2 3 4 5 6 7 8 9 a b c d e f)) {
        $tab{$c} = hex($c);
    }
    
    # parse out each hex character (nibble)
    my @nibbles;
    foreach my $c (split //,$input_str) {
        if (exists $tab{lc($c)}) {
            push @nibbles, $tab{lc($c)};
        }
    }
    # if it's an empty string, just return {}
    if (scalar(@nibbles) == 0) {
        return "{}";
    }
    # make sure we have an even number of nibbles (for bytes)
    if ((scalar(@nibbles)%2)==1) {
        croak("~bytes: I need an even number of hex nibbles");
    }
    # concatenate them into bytes
    my @bytes;
    for (my $i=0; $i < scalar(@nibbles); $i+=2) {
        push @bytes, (($nibbles[$i]<<4) + $nibbles[$i+1]);
    }
    # generate a set of hex constants, e.g. {0xde, 0xad, 0xbe, 0xef}
    my $out = "{";
    foreach my $c (@bytes) {
        $out .= sprintf("0x%02x, ", $c);
    }
    # kill the final ', ', and replace it with a '}'
    $out = substr($out, 0, -2) . "}";
    return $out
}

