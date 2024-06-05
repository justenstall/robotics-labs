#!/usr/bin/env python3

def builtin_binary_to_decimal(num):
	"""
	Function to print binary number from a decimal input using the
	python built-in method bin()
	"""
	new_num = int(num,2)
	# converts a STRING repr of a number with a specific number base
	print( f"Conversion: {new_num}")
	# print the results using a python f-string
	# - intuition is that things between braces {} are statements/variables
	# that get expanded/evaluated when we call the string

# Driver code Example
if __name__ == '__main__':
	builtin_binary_to_decimal('1001')
	builtin_binary_to_decimal('0111')
	builtin_binary_to_decimal('0011')
	builtin_binary_to_decimal('1001')