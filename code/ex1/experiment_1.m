%% Compress and Visualize Audio at Different Bitrates
% This script compresses an audio file at different bitrates using FFmpeg
% and compares their spectrograms.

clear; clc; close all;

%% Step 1: Load Original High-Quality Audio File
[input_audio, fs] = audioread('../../Sample_BeeMoved_96kHz24bit.flac'); % Ensure this file exists in the working directory
disp('Original Audio Loaded Successfully');

%% Step 2: Save as WAV (for FFmpeg compatibility)
wav_filename = 'BeeMoved_Uncompressed.wav';
audiowrite(wav_filename, input_audio, fs);
disp(['Saved Uncompressed File: ', wav_filename]);

%% Step 3: Compress Audio Using FFmpeg
disp('Compressing Audio at Different Bitrates...');
bitrate_list = [320, 160, 96, 64]; % Bitrates in kbps
compressed_files = {};

for i = 1:length(bitrate_list)
    bitrate = bitrate_list(i);
    compressed_filename = sprintf('BeeMoved_%dkbps.mp3', bitrate);
    
    % FFmpeg command to compress WAV to MP3 at specified bitrate
    command = sprintf('ffmpeg -i "%s" -b:a %dk "%s"', wav_filename, bitrate, compressed_filename);
    system(command); % Execute command in MATLAB shell
    
    % Store filenames for visualization
    compressed_files{end+1} = compressed_filename;
    
    fprintf('Saved: %s\n', compressed_filename);
end

disp('Compression Completed.');

%% Step 4: Generate and Compare Spectrograms
figure;
subplot(3,2,1);
spectrogram(input_audio(:,1), 1024, 512, 1024, fs, 'yaxis');
title('Original Audio (Uncompressed WAV)');
colorbar;

% Loop through compressed files and plot spectrograms
for i = 1:length(compressed_files)
    try
        [comp_audio, fs] = audioread(compressed_files{i});
        subplot(3,2,i+1);
        spectrogram(comp_audio(:,1), 1024, 512, 1024, fs, 'yaxis');
        title(sprintf('%d kbps MP3', bitrate_list(i)));
        colorbar;
    catch
        warning(['File not found: ', compressed_files{i}]);
    end
end

sgtitle('Comparison of Spectrograms at Different Bitrates');
disp('Spectrogram comparison complete.');
