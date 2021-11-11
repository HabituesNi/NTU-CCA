clear all
clc

%%
load('data_train.mat')
load('label_train.mat')
load('data_test.mat')

data_label = [data_train,label_train]; % put together
data_test = [data_test];
data_label = shuffling(data_label); % shuffle

train_data = data_label(1:300, 1:33);
train_label = data_label(1:300, 34);
test_data = data_label(301:330, 1:33);
test_label = data_label(301:330, 34);
train_right = [data_train(1:30,1:33)];

% set the RBF network
[net,tr] = newrb(train_data', train_label' ,0 ,1, 30 ,2);

tempresult = sim(net,test_data');
for i=1:30
    if (tempresult(i) < 0) 
        tempresult(i) = -1;
    else 
        tempresult(i) = 1;
    end
end 
tempresult = tempresult';

error = 0;
for j = 1:30
if (tempresult(j)~=test_label(j))
    error = error + 1;
end
end

testresult = sim(net,data_test');
for i=1:21
    if (testresult(i) < 0) 
        result(i) = -1;
    else 
        result(i) = 1;
    end
end 
testresult = testresult';
result = result';

