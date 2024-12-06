#pragma once
#include<QRect>
#include<QDesktopWidget>
#include<QApplication>
#include<string>
#include<iostream>
#include<map>

class GlobalWindowLayout {
public:
	static const GlobalWindowLayout& getInstance() {

		static const GlobalWindowLayout instance;
		return instance;
	}

	const QRect getGeometry(std::string window_name) const {

		std::cout << window_name << std::endl;

		if (geometries.find(window_name) == geometries.end()) {
			std::cout << "ERROR: no global layout for this window" << std::endl;
			std::system("pause");
			std::exit(0);
		}

		return geometries.at(window_name);
	}

private:
	GlobalWindowLayout() {
		QDesktopWidget* desktop_widget = QApplication::desktop();
		QRect desk_rect = desktop_widget->availableGeometry();
		QRect screen_rect = desktop_widget->screenGeometry();

		int screen_width = screen_rect.width();
		int screen_height = screen_rect.height();
		//std::cout << "screen_width: " << screen_width << std::endl;
		//std::cout << "screen_height: " << screen_height << std::endl;
		int margin_x = screen_width / 20;
		int margin_y = screen_height / 8;

		int left = margin_x, top = margin_y;
		geometries[std::string("PanelWindow")] = QRect(left, top,
			screen_width / 8, screen_height / 2);
		left += geometries[std::string("PanelWindow")].width() * 1.2;
		top += 0;

		geometries[std::string("MainWindow")] = QRect(left, top,
			screen_width / 2, screen_height / 2);
		left += geometries[std::string("MainWindow")].width() * 1.2;
		top += 0;

		geometries[std::string("ViewSynthesisWindow")] = QRect(left, top,
			screen_width / 2, screen_height / 2);
		left += geometries[std::string("ViewSynthesisWindow")].width() * 1.2;
		top += 0;

		geometries[std::string("ViewWindow")] = QRect(geometries[std::string("MainWindow")].x() + geometries[std::string("MainWindow")].width() + 30, geometries[std::string("MainWindow")].y(),
			screen_width / 8, geometries[std::string("MainWindow")].height());
	}

	std::map<std::string, QRect> geometries;
};
