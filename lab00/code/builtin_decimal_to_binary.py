#!/usr/bin/env python3

def builtin_decimal_to_binary(num):
	"""
	Function to print binary number from a decimal input using the
	python built-in method bin()
	"""
	new_num = bin(num)
	human_readable = new_num.replace("0b","")
	print( f"Conversion: {new_num} , Human-readable: {human_readable}")

# Driver code Example
if __name__ == '__main__':
	builtin_decimal_to_binary(13)
	builtin_decimal_to_binary(9)
	builtin_decimal_to_binary(3)
	builtin_decimal_to_binary(15)