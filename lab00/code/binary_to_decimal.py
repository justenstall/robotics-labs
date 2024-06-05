#!/usr/bin/env python3

def binary_to_decimal(binary):
	"""
	Function to print decimal number for the input binary
	"""
	decimal = 0
	index = 0
	while(binary != 0):
		dec = binary % 10
		decimal = decimal + dec * pow(2, index)
		binary = binary//10
		index += 1
	print(decimal)

# Driver code Example
if __name__ == '__main__':
	binary_to_decimal(1001)
	binary_to_decimal(111)
	binary_to_decimal(11)
	binary_to_decimal(1001)