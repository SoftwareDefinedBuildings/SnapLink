#include "lib/data/FoundItem.h"
#include "lib/algo/QR.h"
#include "lib/util/Utility.h"
#include <zbar.h>
#include <iostream>

std::vector<FoundItem> QR::QRdetect(const cv::Mat &im) {
  std::vector<FoundItem> results;
  long totalStartTime = Utility::getTime();
 
  std::cout << "Qr extracting\n";
  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  uchar *raw = (uchar *)im.data;
  int width = im.cols;
  int height = im.rows;
  std::cout << "Widith = " << width << " Height = " << height << std::endl;
  zbar::Image image(width, height, "Y800", raw, width * height);

  int n = scanner.scan(image);
  std::cout << "n is " << n << std::endl;
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
       symbol != image.symbol_end(); ++symbol) {
    std::cout << "decoded " << symbol->get_type_name() << " symbol "
              << symbol->get_data() << std::endl;
    // Zbar's coordinate system is topleft = (0,0)
    // x axis is pointing to right
    // y axis is pointing to bottom

    double x0 = symbol->get_location_x(0);
    double x1 = symbol->get_location_x(1);
    double x2 = symbol->get_location_x(2);
    double x3 = symbol->get_location_x(3);
    double y0 = symbol->get_location_y(0);
    double y1 = symbol->get_location_y(1);
    double y2 = symbol->get_location_y(2);
    double y3 = symbol->get_location_y(3);
    double meanX = (x0 + x1 + x2 + x3) / 4;
    double meanY = (y0 + y1 + y2 + y3) / 4;
    double size = (meanX - x0) > 0 ? (meanX - x0) : (x0 - meanX);
    FoundItem item(symbol->get_data(), meanX, meanY, size, width, height);
    std::cout << "Size is " << size << std::endl;
    std::cout << "X is " << meanX << std::endl;
    std::cout << "Y is " << meanY << std::endl;
    results.push_back(item);
  }

  long totalTime = Utility::getTime() - totalStartTime;
  std::cout << "Time QR extract overall " << totalTime << " ms" <<std::endl;
  
  return results;
}