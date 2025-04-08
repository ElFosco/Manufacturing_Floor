

import streamlit as st
from pathlib import Path

import streamlit as st
from pathlib import Path

import streamlit as st
from pathlib import Path

import streamlit as st
from pathlib import Path

import streamlit as st
from pathlib import Path


class ObjectiveSelectorApp:
    class App:
        def __init__(self, nimbus):
            self.nimbus = nimbus

            # Check if the result of nimbus.solve() is already cached in session state
            if 'location_image' not in st.session_state or 'obj_values' not in st.session_state:
                # Only call solve() if it's not already in session state
                location_image, obj_values = self.nimbus.solve()
                st.session_state.location_image = location_image
                st.session_state.obj_values = obj_values
            else:
                location_image = st.session_state.location_image
                obj_values = st.session_state.obj_values

            self.objectives = nimbus.get_objective_names()
            self.options = ["Improve", "Improve *", "OK", "Worsen*", "Other"]
            self.image_path = Path(location_image)
            self.obj_values = obj_values  # Store the objective values here

        def show_image(self):
            st.image(self.image_path, caption="First Solution Found", use_column_width=True)

        def show_objectives(self):
            selections = []
            st.subheader("Choose One Option per Objective")

            # Apply custom style to reduce text size for the objective list
            st.markdown("<style>div.stMarkdown{font-size:12px;}</style>", unsafe_allow_html=True)

            cols = st.columns(len(self.objectives))

            with st.form(key='objective_form', clear_on_submit=False):
                for i, (col, obj, val) in enumerate(zip(cols, self.objectives, self.obj_values)):
                    with col:
                        st.markdown(f"**{obj}**", unsafe_allow_html=True)  # Make the objective names bold but smaller
                        st.markdown(f"<small>Value: `{val}`</small>", unsafe_allow_html=True)  # Smaller value text

                        imp_value = st.number_input("Improve * value", min_value=0.0, key=f"num_improve_{i}", value=0.0)
                        worsen_value = st.number_input("Worsen* value", min_value=0.0, key=f"num_worsen_{i}",
                                                       value=1000.0)

                        choice = st.radio(label="Select preference", options=self.options, index=0, key=f"radio_{i}",
                                          horizontal=True)

                        selections.append(
                            (choice, imp_value, worsen_value))  # Store choice, imp_value, and worsen_value

                # Add a textbox for "No of Solutions"
                no_of_solutions = st.number_input("No of Solutions", min_value=1, step=1, key="no_of_solutions",
                                                  value=10)

                submit_button = st.form_submit_button(label='Submit')

            return selections, submit_button, no_of_solutions

        def calculate(self, selections, no_of_solutions):
            # Create the dictionary to store the data
            result = {'requested': no_of_solutions}
            for i, (choice, imp_value, worsen_value) in enumerate(selections):
                objective_name = self.objectives[i]
                if choice == "Improve *":
                    result[objective_name] = [choice, imp_value]
                elif choice == "Worsen*":
                    result[objective_name] = [choice, worsen_value]
                else:
                    result[objective_name] = [choice]
            return result

        def run(self):
            st.title("Single Image Objective Selector")
            self.show_image()

            selections, submit_button, no_of_solutions = self.show_objectives()

            if submit_button:
                # On button press, calculate and display the result
                result = self.calculate(selections, no_of_solutions)

                # Pass the dictionary to nimbus.compute_new_solutions
                computed_solutions = self.nimbus.compute_new_solutions(result)

                # Display the result
                st.write("Calculated Results:")
                st.json(result)  # Display the result dictionary as JSON for better readability

                # Display the computed solutions returned from Nimbus
                st.write("Computed Solutions:")
                st.json(computed_solutions)  # Assuming compute_new_solutions returns a JSON-like object

