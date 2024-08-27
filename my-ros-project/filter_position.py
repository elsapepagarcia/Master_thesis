# Specify the input file and output file paths
input_file = 'output.txt'
output_file = 'output_2.txt'

# Open the input file for reading and output file for writing
with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        # Check if the word "Position" is in the line
        if 'Position' in line:
            outfile.write(line)

print("Filtered lines have been written to", output_file)
