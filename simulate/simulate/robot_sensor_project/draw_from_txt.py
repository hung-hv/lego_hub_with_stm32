import matplotlib.pyplot as plt

def read_points_from_file(file_name):
    """Read points from a specified file."""
    points = []
    with open(file_name, 'r') as f:
        for line in f:
            # Split the line into x, y, and radius
            x, y, radius = line.strip().split(',')
            points.append((float(x), float(y), float(radius)))
    return points

def plot_points(points):
    """Plot the points with specified coordinates."""
    fig, ax = plt.subplots()
    ax.set_xlim(-600, 600)  # Set x-axis limits from -600 to 600
    ax.set_ylim(-600, 600)  # Set y-axis limits from -600 to 600
    ax.set_title('Drawn Points from File')

    # Draw each point as a circle with radius 1.0
    for x, y, radius in points:
        circle = plt.Circle((x, y), radius, color='blue', fill=True)
        ax.add_artist(circle)

    ax.set_aspect('equal', adjustable='box')  # Maintain the aspect ratio
    plt.grid()
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.axhline(0, color='black',linewidth=0.5, ls='--')  # Add a horizontal line at y=0
    plt.axvline(0, color='black',linewidth=0.5, ls='--')  # Add a vertical line at x=0
    plt.show()

if __name__ == "__main__":
    # Read points from the .txt file
    points = read_points_from_file('drawn_points.txt')
    
    # Plot the points
    plot_points(points)
