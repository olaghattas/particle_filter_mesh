import yaml
from jinja2 import Environment, FileSystemLoader

def add_to_class_after_declaration(file_path, new_code_snippet):
    # Open the file and read its content
    with open(file_path, 'r') as file:
        content = file.readlines()

    # Initialize flags and variables to track the class structure
    class_definition_started = False
    class_start_line = 0
    private_line = 0

    print(f"Reading file: {file_path}")

    # Iterate through each line of the file to find class definition and 'private:' section
    for i, line in enumerate(content):
        print(f"Line {i}: {line.strip()}")  # Debug print to show each line

        # Check if we are inside the class definition
        if "class ParticleFilterNode" in line:
            class_definition_started = True
            class_start_line = i
            print(f"Class definition found at line {i}")

        # Find the private section (assuming it is after the class declaration)
        if 'public:' in line and class_definition_started:
            private_line = i
            print(f"'private:' section found at line {i}")
            break

    if private_line:
        # Insert the new code snippet right after the class definition, before 'private:'
        content.insert(private_line+1, f"    {new_code_snippet}\n")

        # Write the updated content back to the file
        with open(file_path, 'w') as file:
            file.writelines(content)
        print(f"Successfully added the snippet '{new_code_snippet}' after class definition in {file_path}")
    else:
        print("Error: Class definition not found or file format is invalid.")

# Example YAML data for Jinja2 template
with open('yaml_test_init.yaml', 'r') as f:
    class_data = yaml.load(f, Loader=yaml.SafeLoader)

print("Class data loaded from YAML:", class_data)

# Assuming you have a Jinja2 template to render
env = Environment(loader=FileSystemLoader('.'))
template = env.get_template('jinja_template_in_init.jinja2')

cpp_code = template.render(class_data)
print(cpp_code)

# Define the file path and the string to be added (new member variable or method)
file_path = "/home/olagh48652/particle_filter_ws/src/particle_filter_mesh/src/particle_filter_node___.cpp"  # Path to your C++ header file
new_code_snippet = "bool door_garage;"  # The code you want to add

# Call the function to add the new code to the class
# add_to_class_after_declaration(file_path, new_code_snippet)
