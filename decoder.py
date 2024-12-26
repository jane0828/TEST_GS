import numpy as np

def convert_grayscale_to_bmp(input_file, output_file, width, height):
    # Step 1: Open the grayscale file
    with open(input_file, 'rb') as grayscale_file:
        grayscale_data = grayscale_file.read()

    # Step 2: Convert Grayscale 1 bit depth to RGBA 1 bit depth using NumPy
    grayscale_array = np.frombuffer(grayscale_data, dtype=np.uint8)

    # Ensure the correct size for the BMP array
    bmp_array = np.zeros((width * height * 4,), dtype=np.uint8)
    
    # Check if the grayscale array size matches the expected size
    if len(grayscale_array) > width * height:
        raise ValueError("Incorrect size of grayscale data")
    elif len(grayscale_array) < width * height:
    	grayscale_array = np.append(grayscale_array, np.zeros((width * height - len(grayscale_array),), dtype=np.uint8))

    # Set red, green, blue, and alpha channels with the grayscale data
    bmp_array[::4] = grayscale_array
    bmp_array[1::4] = grayscale_array
    bmp_array[2::4] = grayscale_array
    bmp_array[3::4] = 255  # Set alpha channel to 255 (fully opaque)

    # Reshape the BMP array to the original image dimensions
    bmp_array = bmp_array.reshape((height, width, 4))

    # Create BMP header
    bmp_header = bytearray()
    bmp_header.extend(b'BM')  # Signature
    bmp_header.extend((width * height * 4 + 54).to_bytes(4, byteorder='little'))  # File size
    bmp_header.extend(b'\x00\x00\x00\x00')  # Reserved
    bmp_header.extend((54).to_bytes(4, byteorder='little'))  # Data offset
    bmp_header.extend((40).to_bytes(4, byteorder='little'))  # Header size
    bmp_header.extend(width.to_bytes(4, byteorder='little'))  # Image width
    bmp_header.extend(height.to_bytes(4, byteorder='little'))  # Image height
    bmp_header.extend((1).to_bytes(2, byteorder='little'))  # Number of color planes (1)
    bmp_header.extend((32).to_bytes(2, byteorder='little'))  # Bits per pixel (32, 4 channels)
    bmp_header.extend((0).to_bytes(4, byteorder='little'))
    bmp_header.extend((width * height * 4 ).to_bytes(4, byteorder='little'))
    bmp_header.extend(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')

    # Step 3: Write BMP header and image data to output.bmp
    with open(output_file, 'wb') as output_bmp:
        output_bmp.write(bmp_header)
        bmp_array.tofile(output_bmp)

# Example usage
width = 311  # Replace with the actual width of the image
height = 311  # Replace with the actual height of the image
convert_grayscale_to_bmp('comp1.bin', 'compres1.bmp', width, height)
#convert_grayscale_to_bmp('out2.bin', 'reverse_test2.bmp', width, height)
