%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Enhanced Spectrogram Layout with 0.1s X-Axis Spacing
%
% - Loads the original snippet audio (MultiWin_OriginalSnippet.wav)
% - Generates a spectrogram (time vs. frequency)
% - Adjusts x-axis tick spacing to 0.1s intervals
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

% Load the audio file
[signal, fs] = audioread('MultiWin_OriginalSnippet.wav');

% Convert to mono if needed
if size(signal,2) > 1
    signal = mean(signal,2);
end

% Compute spectrogram
window_size = 1024; % Window size for FFT
hop_size = 512; % Hop size for overlap
[S, F, T] = spectrogram(signal, window_size, hop_size, [], fs, 'yaxis');

% Convert power to dB scale
S_dB = 10*log10(abs(S));

% Plot the spectrogram with improved layout
figure('Position', [100, 100, 1200, 600]); % Wider figure for better spacing

imagesc(T, F, S_dB);
axis xy;
colorbar;
caxis([-60 20]); % Adjust contrast for better visibility
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 14, 'FontWeight', 'bold');
title('Spectrogram of MultiWin\_OriginalSnippet.wav', 'FontSize', 16, 'FontWeight', 'bold');

% Set X-Ticks at 0.1s intervals
xticks(0:0.1:max(T));

% Improve layout
grid on;
ylim([0 15000]); % Focus on 0-15 kHz

fprintf('\nSpectrogram generated with X-Tick labels set to 0.1s intervals.\n');
