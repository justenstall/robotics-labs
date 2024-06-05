#!/usr/bin/env python3
import itertools

"""
	This is an example of the first table
	-------------------------------------
	Index     | 0    | 1    | 2
	Binary #  | 0010 | 1010 | 0110
	Decimal # | 2    | 10   | 6
"""

"""
	This is an example of the second table
	--------------------------------------
	original Binary #  | (~)  | (<<1)  | (>>2)
	0010               |
	1010               |
	0110               |
"""

"""
	This is an example of the third table
	--------------------------------------
	Combination  | (&)  | (|)  | (^)
	0010 -- 1010 | 0010 | 1010 |
	0010 -- 0110 | 0010 | 0110 |
	1010 -- 0110 | 0010 | 1110 |
"""

def table1(a, b, c):
	str_index = '{0:<10}'.format('Index')
	def str_idx(num):
		return '{0:<10}'.format(num)
	
	str_binary = '{0:<10}'.format('Binary')
	def str_bin(num):
		return '{0:<10}'.format(bin(num).replace("0b",""))
	
	str_decimal = '{0:<10}'.format('Decimal')
	def str_dec(num):
		return '{0:<10}'.format(int(num))
	
	print('{0:->40}'.format('')) # print a fixed-width string 40 chars long of dashes
	print(f"{str_index} | {str_idx(0)} | {str_idx(1)} | {str_idx(2)}")
	print(f"{str_binary} | {str_bin(a)} | {str_bin(b)} | {str_bin(c)}")
	print(f"{str_decimal} | {str_dec(a)} | {str_dec(b)} | {str_dec(c)}")

def table2(a, b, c):
	str_binary = '{0:<10}'.format('Binary')
	def str_bin(num):
		return '{0:<10}'.format(bin(num).replace("0b",""))
	
	str_inversion = '{0:<12}'.format('Inversion ~')
	def str_inv(num):
		return '{0:<12}'.format(int(~num))
	
	str_left_shift = '{0:<10}'.format('<< 1')
	def str_lshift(num):
		return '{0:<10}'.format(bin(num<<1).replace("0b",""))
	
	str_right_shift = '{0:<10}'.format('>> 2')
	def str_rshift(num):
		return '{0:<10}'.format(bin(num>>2).replace("0b",""))
	
	print(f"{str_binary} | {str_inversion} | {str_left_shift} | {str_right_shift}")
	print('{0:->54}'.format('')) # print a fixed-width string 52 chars long of dashes
	print(f"{str_bin(a)} | {str_inv(a)} | {str_lshift(a)} | {str_rshift(a)}")
	print(f"{str_bin(b)} | {str_inv(b)} | {str_lshift(b)} | {str_rshift(b)}")
	print(f"{str_bin(c)} | {str_inv(c)} | {str_lshift(c)} | {str_rshift(c)}")

def table3(a, b, c):
	str_combination = '{0:<18}'.format('Combination')
	def str_comb(one, two):
		str_one = '{0:>8}'.format(bin(one).replace("0b",""))
		str_two = '{0:<8}'.format(bin(two).replace("0b",""))
		return f"{str_one}--{str_two}"
	
	str_and_op = '{0:<10}'.format('And')
	def str_and(one, two):
		return '{0:<10}'.format(bin(one & two).replace("0b",""))
	
	str_or_op = '{0:<10}'.format('Or')
	def str_or(one, two):
		return '{0:<10}'.format(bin(one | two).replace("0b",""))
	
	str_xor_op = '{0:<10}'.format('Xor')
	def str_xor(one, two):
		return '{0:<10}'.format(bin(one ^ two).replace("0b",""))
	
	print(f"{str_combination} | {str_and_op} | {str_or_op} | {str_xor_op}")
	print('{0:->52}'.format('')) # print a fixed-width string 52 chars long of dashes
	for comb in itertools.combinations([a,b,c], 2):
		one = comb[0]
		two = comb[1]
		print(f"{str_comb(one,two)} | {str_and(one,two)} | {str_or(one,two)} | {str_xor(one,two)}")

# Driver code Example
if __name__ == '__main__':
	b1 = 0b1010
	b2 = 0b0101
	b3 = 0b1101

	table1(b1, b2, b3)
	print("\n")
	table2(b1, b2, b3)
	print("\n")
	table3(b1, b2, b3)