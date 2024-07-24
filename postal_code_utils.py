import pandas as pd

# Load postal code data
postal_code_data = pd.read_csv('SG_Postal.csv')

# Ensure postal_code column is treated as string
postal_code_data['postal_code'] = postal_code_data['postal_code'].astype(str)

def get_coordinates(postal_code):
    # Convert input postal_code to string for comparison
    postal_code = str(postal_code)
    entry = postal_code_data[postal_code_data['postal_code'] == postal_code]
    if entry.empty:
        return None
    return entry.iloc[0]['lat'], entry.iloc[0]['lon']
