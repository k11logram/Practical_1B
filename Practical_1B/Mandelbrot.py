import time
import numpy as np
import matplotlib.pyplot as plt
import logging
import os

def get_color(
    iteration_count, 
    max_iter=1000
    ):
    if iteration_count == max_iter:
        return (0, 0, 0)
    percentage = (iteration_count * 100) // max_iter
    if percentage < 33:
        return (percentage * 7, 0, 255)
    elif percentage < 66:
        return (255, 0, 255 - ((percentage - 33) * 7))
    else:
        return (255, (percentage - 66) * 7, 0)

def generate_mandelbrot_image(
    width, 
    height, 
    max_iter=100
    ):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    mandelbrot_sum = 0
    for y in range(height):
        for x in range(width):
            x0 = (x / width) * 3.5 - 2.5
            y0 = (y / height) * 2.0 - 1.0
            xi = yi = iteration = 0
            while iteration < max_iter and (xi*xi + yi*yi) <= 4:
                tmp = xi*xi - yi*yi + x0
                yi = 2*xi*yi + y0
                xi = tmp
                iteration += 1
            mandelbrot_sum += iteration
            image[y, x] = get_color(iteration, max_iter)
    return mandelbrot_sum, image

def format_table(
    rows, 
    headers
    ):
    # Find max width for each column
    cols = list(zip(*([headers] + rows)))
    col_widths = [max(len(str(item)) for item in col) for col in cols]
    fmt = ' | '.join('{{:{}}}'.format(w) for w in col_widths)
    sep = '-+-'.join('-' * w for w in col_widths)
    table = [fmt.format(*headers), sep]
    for row in rows:
        table.append(fmt.format(*row))
    return '\n'.join(table)

def main():
    os.makedirs("Mandelbrot_results", exist_ok=True)
    filepath = "Mandelbrot_results/"

    # Configure logging
    logging.basicConfig(
        filename=f"{filepath}mandelbrot.log",
        encoding="utf-8",
        filemode="w",
        format="{asctime} - {levelname} - {message}",
        style="{",
        datefmt="%Y-%m-%d %H:%M",
        level=logging.INFO
    )
    
    # Get logger instance
    logger = logging.getLogger(__name__)
    
    image_resolutions = [
        [128, 128],
        [160, 160],
        [192, 192],
        [224, 224],
        [256, 256]
    ]

    logger.info("Mandelbrot Performance Benchmarking")
    
    results = []
    for i in range(len(image_resolutions)):
        width = image_resolutions[i][0]
        height = image_resolutions[i][1]
        start = time.time()
        total, image = generate_mandelbrot_image(width, height)
        elapsed = time.time() - start
        img_file = f"mandelbrot_{width}×{height}.png"
        plt.figure(figsize=(8, 6))
        plt.imshow(image)
        plt.axis('off')
        plt.tight_layout()
        plt.savefig(f"{filepath}{img_file}", bbox_inches='tight', pad_inches=0)

        #plt.plot(image,image_resolutions,label='Mandelbrot Set', color='blue')
        plt.close()
        results.append([
            f"{width}x{height}",
            f"{elapsed:.3f} s",
            str(total),
            img_file
        ])
    # Table headers
    stm_results=[424346, 669809, 966227, 1315085, 1715815]
    plot_results=[]
    resolution=[]
    # Collecting results for plotting
    for i in range(len(results)):
        plot_results.append(int(results[i][2]))
        resolution.append(results[i][0])
    plt.plot(resolution,plot_results,label='Mandelbrot Set(python)', color='blue')
    plt.plot(resolution,stm_results,label='Mandelbrot Set(STM32)', color='Red')
    plt.xlabel('Resolution')
    # naming the y axis
    plt.ylabel('Checksum')

    # giving a title to my graph
    plt.title('STM32 checksum Vs Python Checksum(Reference)')
    plt.show()
    print(results)

    headers = ["Resolution", "Time", "Checksum", "Image File"]
    table = format_table(results, headers)
    print("\nMandelbrot Benchmark Results:")
    print(table)
    logger.info("\n" + table)
    logger.info("Mandelbrot benchmarking completed")

if __name__ == "__main__":
    main()
