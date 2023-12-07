import json
import math
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import NavSatFix

# Class to represent map data from JSON
class JsonDataMap:
    def __init__(self):
        # Initialize attributes to store map segments
        self.rows = None
        self.turns = None
        self.start_path = None
        self.end_path = None
        self.datum = None

    # Helper method to calculate distance between two points
    def calculate_distance(self, point1, point2):
        x1, y1, z1 = point1['head']['position']['x'], point1['head']['position']['y'], point1['head']['position']['z']
        x2, y2, z2 = point2['head']['position']['x'], point2['head']['position']['y'], point2['head']['position']['z']
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance

    # Extract rows data from JSON
    def extract_rows(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Extracting row points and adjusting their coordinates
        rows = [
            {
                'x': point['head']['position']['x'] + json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + json_data['datum']['latitude']
            }
            for point in json_data['points'] if point.get('treatment_area', False)
        ]

        # Extracting x and y coordinates from the adjusted rows
        rows_x = [point['x'] for point in rows]
        rows_y = [point['y'] for point in rows]

        # Calculate total distance for rows
        total_distance = sum(math.sqrt((rows_x[i+1] - rows_x[i])**2 + (rows_y[i+1] - rows_y[i])**2) for i in range(len(rows_x)-1))

        return {'x': rows_x, 'y': rows_y, 'total_distance': total_distance}
    
    # Extract turns data from JSON
    def extract_turns(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding indices of treatment area true points
        treatment_area_indices = [i for i, point in enumerate(json_data['points']) if point.get('treatment_area', False)]

        # Adjusting the turn coordinates coordinates
        turns = [
            {
                'x': point['head']['position']['x'] + json_data['datum']['longitude'],
                'y': point['head']['position']['y'] + json_data['datum']['latitude']
            }
            for i in range(1, len(treatment_area_indices))  # Iterating from the second treatment area point
            for point in json_data['points'][treatment_area_indices[i - 1] + 1: treatment_area_indices[i]]
        ]

        turns_x = [point['x'] for point in turns]
        turns_y = [point['y'] for point in turns]

        # Calculate total distance for turns
        total_distance = sum(math.sqrt((turns_x[i+1] - turns_x[i])**2 + (turns_y[i+1] - turns_y[i])**2) for i in range(len(turns_x)-1))

        return {'x': turns_x, 'y': turns_y, 'total_distance': total_distance}

    # Extract start path data from JSON
    def extract_start_path(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding the index of the first treatment area point
        first_treatment_area_index = next((i for i, point in enumerate(json_data['points']) if point.get('treatment_area', False)), None)

        # Defining the range for the start path
        start_path_x = x[:first_treatment_area_index]
        start_path_y = y[:first_treatment_area_index]

        # Calculate total distance for start path
        total_distance = sum(math.sqrt((start_path_x[i+1] - start_path_x[i])**2 + (start_path_y[i+1] - start_path_y[i])**2) for i in range(len(start_path_x)-1))

        return {'x': start_path_x, 'y': start_path_y, 'total_distance': total_distance}

    # Extract end path data from JSON
    def extract_end_path(self, json_data):
        x = [point['head']['position']['x'] + json_data['datum']['longitude'] for point in json_data['points']]
        y = [point['head']['position']['y'] + json_data['datum']['latitude'] for point in json_data['points']]

        # Finding the index of the last point in the map
        last_point_index = len(json_data['points']) - 1

        # Finding the index of the last treatment area point
        last_treatment_area_index = next((i for i in range(last_point_index, -1, -1) if json_data['points'][i].get('treatment_area', False)), None)

        # Defining the x and y range for the end path
        end_path_x = x[last_treatment_area_index:]
        end_path_y = y[last_treatment_area_index:]

        # Calculate total distance for end path
        total_distance = sum(math.sqrt((end_path_x[i+1] - end_path_x[i])**2 + (end_path_y[i+1] - end_path_y[i])**2) for i in range(len(end_path_x)-1))

        return {'x': end_path_x, 'y': end_path_y, 'total_distance': total_distance}

    # Extract datum (home) coordinates from JSON
    def extract_datum(self, json):
        x = json['datum']['longitude']
        y = json['datum']['latitude']
        return {'x': x, 'y': y}

    # Plot the extracted map data
    def plot_data(self):
        # Scatter plot for rows
        plt.scatter(self.rows['x'],
                    self.rows['y'],
                    color='mediumseagreen',
                    label='Rows',
                    s=10)

        # Scatter plot for turns
        plt.scatter(self.turns['x'],
                    self.turns['y'],
                    color='lightsteelblue',
                    label='Turns',
                    s=10)

        # Scatter plot for start path
        plt.scatter(self.start_path['x'],
                    self.start_path['y'],
                    color='steelblue',
                    label='Start Path',
                    s=10)

        # Scatter plot for end path
        plt.scatter(self.end_path['x'],
                    self.end_path['y'],
                    color='tomato',
                    label='End Path',
                    s=10)

        # Scatter plot for datum (home)
        plt.scatter(self.datum['x'],
                    self.datum['y'],
                    color='black',
                    marker='x',
                    label='Home')

        # Add labels
        plt.xlabel('X Label')
        plt.ylabel('Y Label')

        # Add legend
        plt.legend()

        # Show the plot
        plt.show()


# Class to calculate and store ideal travel times
class IdealTime:
    def __init__(self, json_data, speed):
        self.json_data = json_data
        self.speed = speed
        self.ideal_travel_times = []  # List to store ideal travel times
        self.total_distance_rows = 0  # Variable to store the total distance of all rows

    # Helper method to calculate distance between two points
    def calculate_distance(self, point1, point2):
        x1, y1, z1 = point1['head']['position']['x'], point1['head']['position']['y'], point1['head']['position']['z']
        x2, y2, z2 = point2['head']['position']['x'], point2['head']['position']['y'], point2['head']['position']['z']
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance

    # Calculate and store ideal travel times for each row
    def calculate_and_store_travel_times(self):
        # Extracting relevant information
        points = self.json_data['points']
        rows = []

        # Flag to indicate if a row is currently being processed
        in_row = False

        # Temporary storage for the current row
        current_row = []

        # Iterate through points
        for point in points:
            if point['treatment_area']:
                # Treatment area is true
                if not in_row:
                    # Start of a new row
                    in_row = True
                    current_row = [point]
                else:
                    # Continue adding points to the current row
                    current_row.append(point)
            elif in_row:
                # Treatment area is false, and we were in a row
                in_row = False
                # Check if the row has more than one point (turn)
                if len(current_row) > 1:
                    rows.append(current_row)
                    # Accumulate the total distance of the row
                    total_distance_row = sum(self.calculate_distance(current_row[j], current_row[j + 1]) for j in range(len(current_row) - 1))
                    self.total_distance_rows += total_distance_row

        # Calculate and store travel time for each row
        for i, row in enumerate(rows):
            total_distance = sum(self.calculate_distance(row[j], row[j + 1]) for j in range(len(row) - 1))
            travel_time_seconds = total_distance / self.speed/60  # Travel time in seconds/60 to make minutes
            rounded_travel_time_minutes = round(travel_time_seconds, 2)  # Round to 1 decimal place
            self.ideal_travel_times.append(rounded_travel_time_minutes)

    # Getter method to retrieve total distance of all rows
    def get_total_distance_rows(self):
        return self.total_distance_rows

    # Getter method to retrieve ideal travel times
    def get_ideal_travel_times(self):
        return self.ideal_travel_times


class GPSRecorder:
    def __init__(self, json_data):
        self.json_data = json_data
        self.recorded_data = []

        # Set up ROS subscriber for GPS data
        rospy.Subscriber('gps_head_data', NavSatFix, self.gps_callback)

    def calculate_distance(self, point1, point2):
        x1, y1 = point1['x'], point1['y']
        x2, y2 = point2['x'], point2['y']
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance

    def determine_section(self, gps_data):
        # Reuse the logic from JsonDataMap to determine the section
        x_gps = gps_data.longitude  # Assuming longitude corresponds to X coordinate
        y_gps = gps_data.latitude   # Assuming latitude corresponds to Y coordinate

        # Check if the GPS point is within the rows
        for row_point in self.json_data['points']:
            x_row = row_point['head']['position']['x'] + self.json_data['datum']['longitude']
            y_row = row_point['head']['position']['y'] + self.json_data['datum']['latitude']

            distance = self.calculate_distance({'x': x_gps, 'y': y_gps}, {'x': x_row, 'y': y_row})

            # Set a threshold for considering a point within the rows
            if distance < 0.1:  # Adjust this threshold based on your needs
                return 'rows'

        # Add similar checks for other sections (start path, turns, end path) based on your map data

        # For simplicity, let's assume that the robot is always in the "rows" section
        return 'rows'

    def record_data(self, point_number, section, timestamp):
        self.recorded_data.append({'point_number': point_number, 'section': section, 'timestamp': timestamp})

    def gps_callback(self, gps_data):
        x_gps = gps_data.longitude  # Assuming longitude corresponds to X coordinate
        y_gps = gps_data.latitude   # Assuming latitude corresponds to Y coordinate

        # Iterate through map points to find the closest one
        for point_number, map_point in enumerate(self.json_data['points']):
            x_map = map_point['head']['position']['x'] + self.json_data['datum']['longitude']
            y_map = map_point['head']['position']['y'] + self.json_data['datum']['latitude']

            distance = self.calculate_distance({'x': x_gps, 'y': y_gps}, {'x': x_map, 'y': y_map})

            # Set a threshold for considering a point as passed
            if distance < 0.1:  # Adjust this threshold based on your needs
                section = self.determine_section(gps_data)
                timestamp = rospy.get_time()
                self.record_data(point_number, section, timestamp)

    def determine_section(self, gps_data):
        # Implement logic to determine the section (start path, rows, turns, end path)
        # For simplicity, let's assume that the robot is always in the "rows" section
        return 'rows'

    def record_data(self, point_number, section, timestamp):
        self.recorded_data.append({'point_number': point_number, 'section': section, 'timestamp': timestamp})

# Load JSON data from file
def load_json_data(file_name):
    with open(file_name, 'r') as file:
        return json.load(file)

# Process and plot map data
def process_and_plot_data(json_data, speed_treatment, speed_non_treatment):
    plot_data = JsonDataMap()
    ideal_time = IdealTime(json_data, speed_treatment)

    # Extract and store data
    plot_data.rows = plot_data.extract_rows(json_data)
    plot_data.turns = plot_data.extract_turns(json_data)
    plot_data.start_path = plot_data.extract_start_path(json_data)
    plot_data.end_path = plot_data.extract_end_path(json_data)
    plot_data.datum = plot_data.extract_datum(json_data)

    # Access the total distance for each segment
    total_distance_rows = plot_data.rows['total_distance']
    total_distance_turns = plot_data.turns['total_distance']
    total_distance_start_path = plot_data.start_path['total_distance']
    total_distance_end_path = plot_data.end_path['total_distance']

    # Print or use the total distance information as needed
    print("Total Distance for Rows:", total_distance_rows, "meters")
    print("Total Distance for Turns:", total_distance_turns, "meters")
    print("Total Distance for Start Path:", total_distance_start_path, "meters")
    print("Total Distance for End Path:", total_distance_end_path, "meters")

    # Calculate, print, and store ideal travel times for treatment areas
    ideal_time.calculate_and_store_travel_times()

    # Access the stored ideal travel times for future use
    stored_ideal_times_treatment = ideal_time.get_ideal_travel_times()
    print("Stored Ideal Travel Times for Rows in Minutes:", stored_ideal_times_treatment)

    
    # Access the total distance of rows for treatment areas
    total_distance_rows_treatment = ideal_time.get_total_distance_rows()

    # Calculate and print ideal travel time for treatment areas
    calculate_and_print_ideal_travel_time(total_distance_rows_treatment, speed_treatment, "Treatment Areas")

    # Calculate total distance of non-treatment areas
    points = json_data['points']
    total_distance_all_points = sum(ideal_time.calculate_distance(points[i], points[i + 1]) for i in range(len(points) - 1))

    # Calculate and print ideal travel time for non-treatment areas
    total_distance_non_treatment = total_distance_all_points - total_distance_rows_treatment
    calculate_and_print_ideal_travel_time(total_distance_non_treatment, speed_non_treatment, "Non-Treatment Areas")

    # Plot the data
    plot_data.plot_data()

    return plot_data, ideal_time

# Calculate and print ideal travel time for a given segment
def calculate_and_print_ideal_travel_time(total_distance, speed, label):
    ideal_travel_time = total_distance / speed
    rounded_ideal_travel_time = round(ideal_travel_time / 60, 2)
    print(f"Ideal Travel Time for {label}:", rounded_ideal_travel_time, " Minutes")

# Main function
def main():
    # Load JSON data
    file_name = 'test_map_124.json'
    json_data = load_json_data(file_name)

    # Set speeds for ideal time calculation (in meters per second)
    speed_treatment = 0.23
    speed_non_treatment = 0.5

# Initialize GPSRecorder with the loaded JSON data
    gps_recorder = GPSRecorder(json_data)

    plot_data, ideal_time = process_and_plot_data(json_data, speed_treatment, speed_non_treatment)

if __name__ == "__main__":
    main()
