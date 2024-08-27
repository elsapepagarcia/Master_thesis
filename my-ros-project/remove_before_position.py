# Specify the input file and output file paths
input_file = 'output_2.txt'
output_file = 'output_3.txt'


# Open the input file for reading and output file for writing
with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        # Find the position of the word "Position" in the line
        pos_index = line.find('Position')
        
        # If "Position" is found in the line, extract and write only the numbers
        if pos_index != -1:
            # Extract the part after "Position:"
            data = line[pos_index + len('Position:'):].strip()
            outfile.write(data + '\n')

print("Filtered lines have been written to", output_file)