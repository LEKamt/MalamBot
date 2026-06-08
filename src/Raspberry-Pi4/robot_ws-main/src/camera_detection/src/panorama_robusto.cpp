#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Uso: ./panorama_robusto img1.jpg img2.jpg ..." << endl;
        return -1;
    }

    vector<Mat> images;
    for (int i = 1; i < argc; i++) {
        Mat img = imread(argv[i]);
        if (img.empty()) {
            cout << "No se pudo cargar: " << argv[i] << endl;
            continue;
        }
        
        // Redimensionar para no saturar RAM (importante para RPi 3)
        if (img.cols > 800) {
            resize(img, img, Size(), 0.5, 0.5); 
        }
        images.push_back(img);
    }

    if (images.size() < 2) {
        cout << "Error: Necesitas al menos 2 imagenes validas." << endl;
        return -1;
    }

    cout << "--- Procesando " << images.size() << " fotos ---" << endl;
    
    Mat panorama;
    Ptr<Stitcher> stitcher = Stitcher::create(Stitcher::PANORAMA);
    
    // Ajuste de confianza: 0.3 ayuda cuando el solapamiento no es perfecto
    stitcher->setPanoConfidenceThresh(0.3); 

    Stitcher::Status status = stitcher->stitch(images, panorama);

    if (status != Stitcher::OK) {
        cout << "Fallo en la union. Codigo de error OpenCV: " << int(status) << endl;
        cout << "(Cod 1: Falta solapamiento, Cod 3: No hay suficientes puntos)" << endl;
        return -1;
    }

    imwrite("panorama_perfecto.jpg", panorama);
    cout << "--- ¡Exito! Archivo 'panorama_perfecto.jpg' generado ---" << endl;
    return 0;
}