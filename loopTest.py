if __name__ == '__main__':
	count =0

	while(count<100):
		count = count+1
		print(count)
		if(count ==10):
			print('breaking the loop')
			break

	print('the loop was broken successfully')