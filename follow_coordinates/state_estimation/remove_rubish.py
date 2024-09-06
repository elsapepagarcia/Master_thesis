# Define the start and end words
start_word = "[INFO]"  # Replace with the actual start word
#end_word = "[0m"  # Replace with the actual end word
start_word = "[0m" 
start_word = "[31m"  # Replace with the actual end word
#end_word = "d_err too large, thresholding it!"
#end_word = "m"
#end_word = "[ERROR] -> [ERROR]"
end_word = ""
#start_word = "\n"  # Replace with the actual start word
#end_word = "\n"  # Replace with the actual end word
start_word = "Warning"  # Replace with the actual start word
#end_word = "belief."  # Replace with the actual end word
start_word = "Distance"
#end_word = "position"
start_word = "This"

# Step 1: Read the file
filename = 'modified_data.txt'  # Replace with your .txt file path
with open(filename, 'r') as file:
    lines = file.readlines()

# Step 2: Identify and remove paragraphs
filtered_lines = []
remove_paragraph = False

for line in lines:
    stripped_line = line.strip()
    
    # Check if the paragraph starts with the start_word
    if stripped_line.startswith(start_word):
        remove_paragraph = True
    # Skip empty lines
    if not stripped_line:
        continue
    
    # If not removing, add the line to filtered_lines
    if not remove_paragraph:
        filtered_lines.append(line)
    
    # Check if the paragraph ends with the end_word
    if stripped_line.endswith(end_word):
        remove_paragraph = False

# Step 3: Write the modified content back to the file (or a new file)
with open('modified_data.txt', 'w') as file:  # or use filename to overwrite the original file
    file.writelines(filtered_lines)

print("Paragraphs removed successfully.")
