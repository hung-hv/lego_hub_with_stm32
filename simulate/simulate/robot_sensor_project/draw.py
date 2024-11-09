import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

# Initialize a figure and axis
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
ax1.set_xlim(-600, 600)  # Set x-axis limits from -600 to 600 for the drawing
ax1.set_ylim(-600, 600)  # Set y-axis limits from -600 to 600 for the drawing
ax1.set_title('Draw a Freehand Line - Hold Left Mouse Button')

ax2.set_xlim(-600, 600)  # Set x-axis limits for the polynomial graph
ax2.set_ylim(-600, 600)  # Set y-axis limits for the polynomial graph
ax2.set_title('Fitted Polynomial Curve')

# Lists to store the x and y coordinates of the points
x_data = []
y_data = []

# Function to handle mouse button press
def on_press(event):
    if event.button == 1:  # Left mouse button
        x_data.append(event.xdata)
        y_data.append(event.ydata)

# Function to handle mouse movement
def on_motion(event):
    if event.button == 1:  # Left mouse button
        x_data.append(event.xdata)
        y_data.append(event.ydata)
        ax1.plot(x_data, y_data, color='blue')
        plt.draw()

# Function to handle mouse button release
def on_release(event):
    if event.button == 1:  # Left mouse button
        if len(x_data) >= 2:
            polynomial_eq, equation = fit_polynomial(x_data, y_data)
            print("Fitted Polynomial Equation:", equation)
            draw_polynomial_curve(polynomial_eq)

# Function to fit a polynomial to the points and return the equation
def fit_polynomial(x_data, y_data, degree=5):
    # Fit a polynomial of specified degree
    coefficients = np.polyfit(x_data, y_data, degree)
    polynomial = np.poly1d(coefficients)
    
    # Generate the equation in a readable format
    equation_terms = []
    for i, coeff in enumerate(coefficients):
        term = f"{coeff:.2f}x^{degree - i}" if degree - i > 0 else f"{coeff:.2f}"
        equation_terms.append(term)
    
    equation = "y = " + " + ".join(equation_terms).replace("x^1", "x").replace("x^0", "")
    return polynomial, equation  # Return the polynomial object and equation for drawing

# Function to draw the polynomial curve on the second graph
def draw_polynomial_curve(polynomial):
    # Generate x values for plotting the polynomial
    x_vals = np.linspace(-600, 600, 400)
    y_vals = polynomial(x_vals)
    
    # Clear the previous plot and draw the new polynomial
    ax2.clear()
    ax2.set_xlim(-600, 600)
    ax2.set_ylim(-600, 600)
    ax2.set_title('Fitted Polynomial Curve')
    ax2.plot(x_vals, y_vals, color='red')
    ax2.axhline(0, color='black', linewidth=0.5, ls='--')
    ax2.axvline(0, color='black', linewidth=0.5, ls='--')
    ax2.grid(True)
    plt.draw()

# Function to save points and equation to .txt files
def save_points_and_equation(event):
    # Save drawn points to drawn_points.txt
    with open('drawn_points.txt', 'w') as f:
        for x, y in zip(x_data, y_data):
            f.write(f"{x:.2f},{y:.2f},1.0\n")  # Write x, y, and radius (1.0 mm)
    print("Points saved to drawn_points.txt")
    
    # Save polynomial equation to a text file
    if len(x_data) >= 2:
        polynomial_eq, equation = fit_polynomial(x_data, y_data)
        with open('polynomial_equation.txt', 'w') as f_eq:
            f_eq.write(equation + '\n')  # Save the equation as a string
        print("Equation saved to polynomial_equation.txt")

# Function to clear the contents of the .txt file and reset the graphs
def clear_file(event):
    with open('drawn_points.txt', 'w') as f:
        f.write("")  # Write an empty string to clear the file
    print("Contents of drawn_points.txt cleared")
    
    # Clear the drawn points and the polynomial
    x_data.clear()
    y_data.clear()
    
    ax1.clear()
    ax1.set_xlim(-600, 600)
    ax1.set_ylim(-600, 600)
    ax1.set_title('Draw a Freehand Line - Hold Left Mouse Button')

    ax2.clear()
    ax2.set_xlim(-600, 600)
    ax2.set_ylim(-600, 600)
    ax2.set_title('Fitted Polynomial Curve')
    
    plt.draw()  # Redraw the cleared plots

# Create "Save" button
ax_save = plt.axes([0.8, 0.01, 0.1, 0.05])  # Position for the Save button
button_save = Button(ax_save, 'Save')
button_save.on_clicked(save_points_and_equation)

# Create "Clear" button
ax_clear = plt.axes([0.65, 0.01, 0.1, 0.05])  # Position for the Clear button
button_clear = Button(ax_clear, 'Clear')
button_clear.on_clicked(clear_file)

# Connect the event handlers to the figure
cid_press = fig.canvas.mpl_connect('button_press_event', on_press)
cid_motion = fig.canvas.mpl_connect('motion_notify_event', on_motion)
cid_release = fig.canvas.mpl_connect('button_release_event', on_release)

# Show the plot
plt.show()
