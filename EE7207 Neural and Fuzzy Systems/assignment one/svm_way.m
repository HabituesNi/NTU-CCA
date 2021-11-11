clear all
clc

%%
load('data_train.mat')
load('label_train.mat')
load('data_test.mat')

data_label = [data_train,label_train]; % put together
data_test = [data_test];
data_label = shuffling(data_label); % shuffle
train_right = [data_train(1:30,1:33)];

train_data = data_label(1:330, 1:33);
train_label = data_label(1:330, 34);
test_data = data_label(301:330, 1:33);
test_label = data_label(301:330, 34);


% set the SVM network
mdl = fitcsvm(train_data, train_label);
fitprosperity = crossval(mdl);
classloss = kfoldLoss(fitprosperity);
[~,score] = predict(mdl, data_test);
[label, scorePred] = kfoldPredict(fitprosperity);
tempresult = score(:,2);
result = zeros(21,1);
for i=1:21
    if tempresult(i,1)<0
        result(i,1) = -1;
    else
        result(i,1) = 1;
    end
end


