#!/usr/bin/env python3

def decimal_to_binary(num):
	"""
	Function to print binary number for the input decimal using recursion
	"""
	if num > 1:
		# divide with integral result (discard remainder)
		decimal_to_binary(num // 2)
	print(num % 2, end = '')

# Driver code Example
if __name__ == '__main__':
	decimal_to_binary(13)
	print("") # Here just for formatting the output
	decimal_to_binary(9)
	print("") # Here just for formatting the output
	decimal_to_binary(3)
	print("") # Here just for formatting the output
	decimal_to_binary(15)
	print("") # Here just for formatting the output