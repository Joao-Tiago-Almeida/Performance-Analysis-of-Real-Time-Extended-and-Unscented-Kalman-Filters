clear all
close all
clc

%%
imshow(imread('yellowlily.jpg'))
i = 1;
finished = 'NO';
i = 1;
while strcmpi(finished,'NO')
  h = drawfreehand('Closed',false,'FaceAlpha',0,'InteractionsAllowed','none');
  finished = questdlg('Finished?', ...
      'confirmation', ...
      'YES', 'NO', 'UNDO', 'NO');
  if strcmpi(finished, 'UNDO')
      delete(h)
      finished = 'NO';
  else
      xy{i}  = h.Position;
      i = i + 1;
  end
end