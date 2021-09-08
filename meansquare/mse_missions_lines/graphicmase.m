name = "mse_allattacks1";
file = name + ".csv.mat";
mse = load(file);
mse_line = getfield(mse,'mse_all');
fig = plot(1:length(mse_line), mse_line);
savefile = "fig_"+ name +".png";
saveas(fig, savefile)