import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
import cv2
import numpy as np
import os
import csv

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class PilotNet(nn.Module):
    def __init__(self):
        super(PilotNet, self).__init__()
        self.conv_layers = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3), nn.ReLU(),
            nn.Conv2d(64, 64, 3), nn.ReLU(),
        )
        self.fc_layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(6656, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, 2)
        )

    def forward(self, x):
        x = self.conv_layers(x)
        x = self.fc_layers(x)
        return x

class FastCorridorDataset(Dataset):
    def __init__(self, data_dir, transform=None):
        self.transform = transform
        self.images = []
        self.labels = []
        
        csv_path = os.path.join(data_dir, 'driving_log.csv')
        print("💾 데이터를 RAM으로 미리 로딩 중... (이 작업은 한 번만 수행됩니다)")
        
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                img_path = os.path.join(data_dir, row['image_path'])
                image = cv2.imread(img_path)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                # 메모리에 원본 이미지 저장
                self.images.append(image)
                self.labels.append([float(row['steer']) / 128.0, float(row['throttle']) / 128.0])
        
        print(f"✅ {len(self.images)}장 로딩 완료!")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        image = self.images[idx]
        label = torch.FloatTensor(self.labels[idx])
        
        if self.transform:
            image = self.transform(image)
        return image, label

# --- 설정 및 실행 ---
DATA_DIR = "extracted_data"
BATCH_SIZE = 32 # RTX 5070의 위력을 위해 배치 사이즈 상향
EPOCHS = 200
LEARNING_RATE = 0.0005

transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((120, 160)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

dataset = FastCorridorDataset(DATA_DIR, transform=transform)
# num_workers와 pin_memory를 활용해 병목 제거
dataloader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=4, pin_memory=True)

model = PilotNet().to(device)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

print("🏎️ RTX 5070 가속 학습 시작!")
for epoch in range(EPOCHS):
    model.train()
    running_loss = 0.0
    for images, labels in dataloader:
        images, labels = images.to(device), labels.to(device)
        
        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        running_loss += loss.item()
    
    if (epoch + 1) % 10 == 0:
        print(f"Epoch [{epoch+1}/{EPOCHS}], Loss: {running_loss/len(dataloader):.6f}")

torch.save(model.state_dict(), 'model.pth')
print("🏁 학습 끝!")