// QRcodeScanner.hpp (修改后)
#include <iostream>
#include <opencv2/opencv.hpp>
#include <zbar.h>

// 条码识别基类
class BarcodeScanner
{
public:
    virtual std::string getBarcode(cv::Mat& raw_image) = 0;
};

// ZBar 实现，支持条形码识别
class ZbarBarcodeScanner : public BarcodeScanner
{
private:
    zbar::ImageScanner* image_scanner;
    cv::Mat gray_image;

public:
    ZbarBarcodeScanner() : image_scanner(new zbar::ImageScanner)
    {
        // 禁用全部，再单独启用需要的条码类型（或直接全开）
        image_scanner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        // 启用常见一维条形码
        image_scanner->set_config(zbar::ZBAR_EAN13, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_EAN8, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_UPCA, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_UPCE, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_ISBN10, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_ISBN13, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_CODE39, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_CODE93, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_I25, zbar::ZBAR_CFG_ENABLE, 1);
        image_scanner->set_config(zbar::ZBAR_DATABAR, zbar::ZBAR_CFG_ENABLE, 1);
        // 若也要识别 QR 码，可以额外开启 ZBAR_QRCODE
    }

    ~ZbarBarcodeScanner()
    {
        delete image_scanner;
    }

    std::string getBarcode(cv::Mat& raw_image) override
    {
        if (raw_image.empty())
        {
            std::cerr << "EMPTY IMAGE!" << std::endl;
            return "";
        }

        cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);

        zbar::Image zbar_image(gray_image.cols, gray_image.rows, "Y800",
                               gray_image.data, gray_image.cols * gray_image.rows);

        image_scanner->scan(zbar_image);

        std::string result;
        for (zbar::SymbolIterator symbol = zbar_image.symbol_begin();
             symbol != zbar_image.symbol_end(); ++symbol)
        {
            result = symbol->get_data();
            if (!result.empty())
            {
                break;  // 只取第一个识别结果
            }
        }

        // 清理 ZBar 图像
        zbar_image.set_data(nullptr, 0);

        return result;
    }
};