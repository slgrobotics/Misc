
void performFFT()
{
  // Prepare FFT calculation data (1.7ms)
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
    //vReal[i] = wave[i]; //(wave[i] - 2048) * 3.3 / 4096.0; // convert to voltage
    vReal[i] = wave[i] * 3.3 / 4096.0; // convert to voltage
    vImag[i] = 0;
  }
 
  // Calculate FFT (33ms)
  FFT.Windowing(vReal, SAMPLE_BUFFER_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Apply window function (Hamming)
 
  FFT.Compute(vReal, vImag, SAMPLE_BUFFER_SIZE, FFT_FORWARD); // FFT
 
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_BUFFER_SIZE); // Calculate absolute value
}