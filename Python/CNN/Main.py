import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageEnhance, ImageOps, ImageFilter
import random
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import classification_report, confusion_matrix
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import warnings
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
from scipy.ndimage import rotate, zoom, affine_transform
import math
import json
from datetime import datetime

warnings.filterwarnings('ignore')

# ============================================
# 1. CONFIGURAZIONE
# ============================================
def setup_gpu_memory(max_percentage=0.8):
    if not torch.cuda.is_available():
        print("⚠️  GPU non disponibile, usando CPU")
        return None
    
    print("\n" + "="*50)
    print("CONFIGURAZIONE MEMORIA GPU")
    print("="*50)
    
    torch.cuda.empty_cache()
    torch.cuda.set_per_process_memory_fraction(max_percentage, device=0)
    
    total = torch.cuda.get_device_properties(0).total_memory
    allocated = torch.cuda.memory_allocated(0)
    
    print(f"Dispositivo: {torch.cuda.get_device_name(0)}")
    print(f"Memoria totale: {total / 1e9:.2f} GB")
    
    def monitor_gpu(step=""):
        alloc = torch.cuda.memory_allocated(0) / 1e9
        max_mem = total * max_percentage / 1e9
        perc = (alloc / max_mem) * 100 if max_mem > 0 else 0
        if step:
            print(f"   {step}GPU: {alloc:.3f}/{max_mem:.2f} GB ({perc:.1f}%)")
        return alloc, max_mem, perc
    
    print("="*50)
    return monitor_gpu

def check_system():
    print("=" * 50)
    print("DIAGNOSTICA PYTORCH")
    print("=" * 50)
    print(f"Python: {sys.version}")
    print(f"Torch version: {torch.__version__}")
    print(f"Torch CUDA disponibile: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"Torch CUDA Version: {torch.version.cuda}")
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Dispositivo in uso: {device}")
    return device

# ============================================
# 2. CONFIGURAZIONE ROTAZIONI
# ============================================
class Config:
    DATASET_PATH = "Images\Images"  
    IMG_HEIGHT = 64
    IMG_WIDTH = 64
    CHANNELS = 1
    INVERT_COLORS = True
    
    # AUGMENTATION PER ROTAZIONI
    AUGMENT_ROTATION = True
    MAX_ROTATION_ANGLE = 45  # Aumentato a ±45°
    ROTATION_PROBABILITY = 0.7
    
    # ALTRE AUGMENTATIONS
    AUGMENT_ZOOM = True
    ZOOM_RANGE = [0.8, 1.2]
    AUGMENT_SHEAR = True
    SHEAR_RANGE = 15
    AUGMENT_BRIGHTNESS = True
    BRIGHTNESS_RANGE = [0.8, 1.2]
    AUGMENT_CONTRAST = True
    CONTRAST_RANGE = [0.8, 1.2]
    AUGMENT_NOISE = True
    NOISE_STRENGTH = 0.02
    AUGMENT_BLUR = True
    BLUR_RADIUS = 0.5
    
    # Parametri modello
    BATCH_SIZE = 64
    EPOCHS = 80
    NUM_CLASSES = 26
    VALIDATION_SPLIT = 0.2
    TEST_SPLIT = 0.1
    MAX_IMAGES_PER_CLASS = 0
    NUM_WORKERS = 8
    GPU_MEMORY_LIMIT = 0.8
    CONFUSION_CORRECTION = True
    TEST_MULTIPLE_ORIENTATIONS = True
    ORIENTATION_ANGLES = [0, 45, 90, 135, 180, 225, 270, 315]
    EARLY_STOPPING_PATIENCE = 15
    
    MODEL_SAVE_PATH = "letter_recognition_model_rotated.pth"
    LABEL_ENCODER_PATH = "label_encoder_rotated.npy"

config = Config()
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# ============================================
# 3. AUGMENTATION SENZA ALBUMENTATIONS
# ============================================
class PILAugmenter:
    """Augmentation usando solo PIL e scipy"""
    
    @staticmethod
    def random_rotation(image_array, max_angle=45):
        """Rotazione casuale"""
        if random.random() < config.ROTATION_PROBABILITY:
            angle = random.uniform(-max_angle, max_angle)
            if config.CHANNELS == 1:
                rotated = rotate(image_array[0], angle, reshape=False, mode='nearest')
                return np.expand_dims(rotated, axis=0)
            else:
                rotated = np.zeros_like(image_array)
                for c in range(image_array.shape[0]):
                    rotated[c] = rotate(image_array[c], angle, reshape=False, mode='nearest')
                return rotated
        return image_array
    
    @staticmethod
    def random_zoom(image_array, zoom_range=[0.8, 1.2]):
        """Zoom casuale"""
        if random.random() < 0.3:
            scale = random.uniform(zoom_range[0], zoom_range[1])
            
            if config.CHANNELS == 1:
                h, w = image_array.shape[1], image_array.shape[2]
                new_h, new_w = int(h * scale), int(w * scale)
                
                # Crea immagine PIL
                img = Image.fromarray((image_array[0] * 255).astype(np.uint8))
                img = img.resize((new_w, new_h), Image.LANCZOS)
                
                # Rimetti su canvas originale
                result = Image.new('L', (w, h), color=255)
                x_offset = (w - new_w) // 2
                y_offset = (h - new_h) // 2
                result.paste(img, (x_offset, y_offset))
                
                zoomed = np.array(result) / 255.0
                return np.expand_dims(zoomed, axis=0)
        return image_array
    
    @staticmethod
    def random_shear(image_array, max_shear=15):
        """Shear/distorsione"""
        if random.random() < 0.3:
            shear_x = math.radians(random.uniform(-max_shear, max_shear))
            shear_y = math.radians(random.uniform(-max_shear, max_shear))
            
            if config.CHANNELS == 1:
                matrix = np.array([[1, shear_x, 0],
                                  [shear_y, 1, 0]])
                transformed = affine_transform(image_array[0], matrix, order=1)
                return np.expand_dims(transformed, axis=0)
        return image_array
    
    @staticmethod
    def random_brightness_contrast(image_array):
        """Variazione luminosità/contrasto"""
        if config.CHANNELS == 1:
            img = Image.fromarray((image_array[0] * 255).astype(np.uint8))
            
            # Luminosità
            if random.random() < 0.3:
                enhancer = ImageEnhance.Brightness(img)
                factor = random.uniform(config.BRIGHTNESS_RANGE[0], config.BRIGHTNESS_RANGE[1])
                img = enhancer.enhance(factor)
            
            # Contrasto
            if random.random() < 0.3:
                enhancer = ImageEnhance.Contrast(img)
                factor = random.uniform(config.CONTRAST_RANGE[0], config.CONTRAST_RANGE[1])
                img = enhancer.enhance(factor)
            
            result = np.array(img) / 255.0
            return np.expand_dims(result, axis=0)
        return image_array
    
    @staticmethod
    def random_noise(image_array, strength=0.02):
        """Rumore gaussiano"""
        if random.random() < 0.2:
            noise = np.random.normal(0, strength, image_array.shape)
            result = np.clip(image_array + noise, 0, 1)
            return result
        return image_array
    
    @staticmethod
    def random_blur(image_array, radius=0.5):
        """Sfocatura leggera"""
        if random.random() < 0.2:
            if config.CHANNELS == 1:
                img = Image.fromarray((image_array[0] * 255).astype(np.uint8))
                img = img.filter(ImageFilter.GaussianBlur(radius=radius))
                result = np.array(img) / 255.0
                return np.expand_dims(result, axis=0)
        return image_array
    
    @staticmethod
    def apply_all_augmentations(image_array):
        """Applica tutte le augmentations"""
        if not config.AUGMENT_ROTATION:
            return image_array
        
        # Rotazione (sempre prima)
        image_array = PILAugmenter.random_rotation(image_array, config.MAX_ROTATION_ANGLE)
        
        # Altre augmentations in ordine casuale
        augmentations = [
            lambda x: PILAugmenter.random_zoom(x, config.ZOOM_RANGE),
            lambda x: PILAugmenter.random_shear(x, config.SHEAR_RANGE),
            lambda x: PILAugmenter.random_brightness_contrast(x),
            lambda x: PILAugmenter.random_noise(x, config.NOISE_STRENGTH),
            lambda x: PILAugmenter.random_blur(x, config.BLUR_RADIUS)
        ]
        
        # Applica 2-3 augmentations casuali
        num_augmentations = random.randint(2, 4)
        selected_augmentations = random.sample(augmentations, num_augmentations)
        
        for aug_func in selected_augmentations:
            if random.random() < 0.5:  # 50% di probabilità per ciascuna
                image_array = aug_func(image_array)
        
        return image_array

# ============================================
# 4. PREPROCESSING CON ROTAZIONE
# ============================================
def enhanced_preprocessing_with_rotation(img_path, invert_colors=True, test_rotation=False, angle=0):
    try:
        img = Image.open(img_path).convert('L')
        
        if test_rotation:
            img = img.rotate(angle, expand=False, resample=Image.BICUBIC)
        
        original_size = img.size
        img_array = np.array(img)
        
        # Trova bounding box
        if img_array.mean() > 128:
            threshold = img_array.mean() * 0.7
            mask = img_array < threshold
        else:
            threshold = img_array.mean() * 1.3
            mask = img_array > threshold
        
        rows = np.any(mask, axis=1)
        cols = np.any(mask, axis=0)
        
        if np.any(rows) and np.any(cols):
            ymin, ymax = np.where(rows)[0][[0, -1]]
            xmin, xmax = np.where(cols)[0][[0, -1]]
            
            margin = 5
            ymin = max(0, ymin - margin)
            ymax = min(img_array.shape[0], ymax + margin)
            xmin = max(0, xmin - margin)
            xmax = min(img_array.shape[1], xmax + margin)
            
            img_cropped = img.crop((xmin, ymin, xmax, ymax))
        else:
            img_cropped = img
        
        width, height = img_cropped.size
        if width == 0 or height == 0:
            img_cropped = img
            width, height = img_cropped.size
        
        target_size = config.IMG_WIDTH
        scale = target_size / max(width, height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        
        img_resized = img_cropped.resize((new_width, new_height), Image.LANCZOS)
        
        if config.CHANNELS == 1:
            img_final = Image.new('L', (target_size, target_size), color=255)
        else:
            img_final = Image.new('RGB', (target_size, target_size), color=(255, 255, 255))
        
        offset = ((target_size - new_width) // 2, (target_size - new_height) // 2)
        img_final.paste(img_resized, offset)
        
        img_array = np.array(img_final)
        
        if invert_colors:
            img_array = 255 - img_array
        
        img_array = img_array.astype('float32') / 255.0
        
        if config.CHANNELS == 1:
            img_array = np.expand_dims(img_array, axis=0)
        else:
            img_array = np.transpose(img_array, (2, 0, 1))
        
        return img_array, original_size, angle if test_rotation else 0
        
    except Exception as e:
        print(f"  [ERROR] Preprocessing fallito: {e}")
        return None, None, 0

# ============================================
# 5. DATASET CON AUGMENTATION
# ============================================
class AugmentedLetterDataset(Dataset):
    def __init__(self, images, labels, augment=True):
        self.images = torch.FloatTensor(images)
        self.labels = torch.LongTensor(labels)
        self.augment = augment
        self.augmenter = PILAugmenter()
    
    def __len__(self):
        return len(self.images)
    
    def __getitem__(self, idx):
        img = self.images[idx].clone()
        label = self.labels[idx]
        
        if self.augment:
            img_np = img.numpy()
            img_np = self.augmenter.apply_all_augmentations(img_np)
            img = torch.FloatTensor(img_np)
        
        return img, label

# ============================================
# 6. MODELLO ROBUSTO ALLE ROTAZIONI
# ============================================
class RotationRobustCNN(nn.Module):
    def __init__(self, input_shape, num_classes):
        super(RotationRobustCNN, self).__init__()
        in_channels = input_shape[0]
        
        self.block1 = nn.Sequential(
            nn.Conv2d(in_channels, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout2d(0.25),
        )
        
        self.block2 = nn.Sequential(
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout2d(0.25),
        )
        
        self.block3 = nn.Sequential(
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),  # Pooling adattivo per rotazioni
            nn.Dropout2d(0.3),
        )
        
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(256 * 4 * 4, 512),
            nn.ReLU(inplace=True),
            nn.BatchNorm1d(512),
            nn.Dropout(0.5),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.BatchNorm1d(256),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes),
        )
    
    def forward(self, x):
        x = self.block1(x)
        x = self.block2(x)
        x = self.block3(x)
        x = self.classifier(x)
        return x

# ============================================
# 7. TRAINING CON TEST ROTAZIONI
# ============================================
def train_with_rotation_testing(model, train_loader, val_loader, label_encoder, monitor_gpu=None):
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.AdamW(model.parameters(), lr=0.001, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)
    
    best_val_loss = float('inf')
    patience_counter = 0
    best_model_state = None
    
    history = {
        'train_loss': [], 'train_acc': [],
        'val_loss': [], 'val_acc': [],
        'rotation_acc': []
    }
    
    print("\n🚀 INIZIO TRAINING CON TEST ROTAZIONI...")
    
    for epoch in range(config.EPOCHS):
        if monitor_gpu and (epoch % 5 == 0):
            monitor_gpu(f"Epoch {epoch+1} - ")
        
        # Training
        model.train()
        train_loss = 0.0
        train_correct = 0
        train_total = 0
        
        for batch_idx, (inputs, targets) in enumerate(train_loader):
            inputs, targets = inputs.to(device), targets.to(device)
            
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            
            # Aggiungi loss su immagini ruotate (per robustezza)
            if batch_idx % 10 == 0:
                angles = torch.FloatTensor(inputs.size(0)).uniform_(-30, 30).to(device)
                rotated_inputs = batch_rotate(inputs, angles)
                rotated_outputs = model(rotated_inputs)
                rotation_loss = criterion(rotated_outputs, targets)
                loss = loss + 0.3 * rotation_loss
            
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            
            train_loss += loss.item()
            _, predicted = outputs.max(1)
            train_total += targets.size(0)
            train_correct += predicted.eq(targets).sum().item()
        
        train_loss = train_loss / len(train_loader)
        train_acc = 100. * train_correct / train_total
        
        # Validation
        model.eval()
        val_loss = 0.0
        val_correct = 0
        val_total = 0
        
        with torch.no_grad():
            for inputs, targets in val_loader:
                inputs, targets = inputs.to(device), targets.to(device)
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                
                val_loss += loss.item()
                _, predicted = outputs.max(1)
                val_total += targets.size(0)
                val_correct += predicted.eq(targets).sum().item()
        
        val_loss = val_loss / len(val_loader)
        val_acc = 100. * val_correct / val_total
        
        # Test rotazioni (ogni 5 epoch)
        rotation_acc = 0
        if epoch % 5 == 0:
            rotation_acc = test_rotation_robustness(model, val_loader)
            history['rotation_acc'].append(rotation_acc)
        
        history['train_loss'].append(train_loss)
        history['train_acc'].append(train_acc)
        history['val_loss'].append(val_loss)
        history['val_acc'].append(val_acc)
        
        scheduler.step(val_loss)
        
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
            best_model_state = model.state_dict().copy()
        else:
            patience_counter += 1
        
        if (epoch + 1) % 2 == 0:
            rotation_info = f" | Rot Acc: {rotation_acc:.1f}%" if rotation_acc > 0 else ""
            print(f"Epoch {epoch+1:3d}/{config.EPOCHS} | "
                  f"Train Acc: {train_acc:.2f}% | Val Acc: {val_acc:.2f}%"
                  f"{rotation_info}")
        
        if patience_counter >= config.EARLY_STOPPING_PATIENCE:
            print(f"\n⏹️  Early stopping at epoch {epoch+1}")
            break
    
    if best_model_state is not None:
        model.load_state_dict(best_model_state)
    
    return history, model

def batch_rotate(batch, angles):
    """Ruota batch di immagini"""
    rotated_batch = []
    for img, angle in zip(batch, angles):
        img_np = img.cpu().numpy()
        if len(img_np.shape) == 3:
            rotated = np.zeros_like(img_np)
            for c in range(img_np.shape[0]):
                rotated[c] = rotate(img_np[c], angle.item(), reshape=False, mode='nearest')
        else:
            rotated = rotate(img_np, angle.item(), reshape=False, mode='nearest')
        rotated_batch.append(torch.FloatTensor(rotated).to(device))
    return torch.stack(rotated_batch)

def test_rotation_robustness(model, data_loader, num_samples=200):
    """Testa robustezza a diverse rotazioni"""
    model.eval()
    test_angles = [-45, -30, -15, 0, 15, 30, 45]
    results = {}
    
    for angle in test_angles:
        correct = 0
        total = 0
        
        with torch.no_grad():
            for batch_idx, (inputs, targets) in enumerate(data_loader):
                if batch_idx * data_loader.batch_size >= num_samples:
                    break
                
                inputs, targets = inputs.to(device), targets.to(device)
                rotated_inputs = batch_rotate(inputs, torch.full((inputs.size(0),), angle).to(device))
                outputs = model(rotated_inputs)
                _, predicted = outputs.max(1)
                
                total += targets.size(0)
                correct += predicted.eq(targets).sum().item()
        
        accuracy = 100. * correct / total if total > 0 else 0
        results[angle] = accuracy
    
    print(f"\n📊 TEST ROTAZIONI:")
    for angle, acc in results.items():
        print(f"  Angolo {angle:3d}°: {acc:.1f}%")
    
    return np.mean(list(results.values())[1:-1])  # Media senza ±45°

# ============================================
# 8. RICONOSCITORE MULTI-ORIENTAMENTO
# ============================================
class MultiOrientationRecognizer:
    def __init__(self, model_path=None, encoder_path=None):
        self.device = device
        self.label_encoder = LabelEncoder()
        self.label_encoder.classes_ = np.load(
            encoder_path or config.LABEL_ENCODER_PATH, 
            allow_pickle=True
        )
        
        input_shape = (config.CHANNELS, config.IMG_HEIGHT, config.IMG_WIDTH)
        self.model = RotationRobustCNN(input_shape, config.NUM_CLASSES)
        
        model_to_load = model_path or config.MODEL_SAVE_PATH
        if torch.cuda.is_available():
            self.model.load_state_dict(torch.load(model_to_load))
        else:
            self.model.load_state_dict(torch.load(model_to_load, map_location=torch.device('cpu')))
        
        self.model.to(self.device)
        self.model.eval()
    
    def predict_with_best_orientation(self, image_path):
        """Predice testando multiple orientazioni"""
        results = []
        
        for angle in config.ORIENTATION_ANGLES:
            try:
                # Carica e ruota immagine
                img_array, _, _ = enhanced_preprocessing_with_rotation(
                    image_path, 
                    invert_colors=config.INVERT_COLORS,
                    test_rotation=True,
                    angle=angle
                )
                
                if img_array is not None:
                    # Predici
                    prediction = self._predict_single(img_array)
                    if prediction:
                        prediction['angle'] = angle
                        results.append(prediction)
                        
            except Exception as e:
                continue
        
        if not results:
            return None
        
        # Trova il risultato migliore
        best_result = max(results, key=lambda x: x['confidence'])
        
        # Controlla se c'è consenso tra multiple orientazioni
        letter_votes = {}
        for result in results:
            letter = result['predicted_letter']
            if letter not in letter_votes:
                letter_votes[letter] = []
            letter_votes[letter].append(result['confidence'])
        
        for letter, confidences in letter_votes.items():
            if len(confidences) >= 3:  # Almeno 3 orientamenti concordano
                avg_confidence = np.mean(confidences)
                if avg_confidence > best_result['confidence'] * 0.9:
                    best_result = {
                        'predicted_letter': letter,
                        'confidence': avg_confidence,
                        'angle': angle,
                        'consensus': True,
                        'vote_count': len(confidences)
                    }
                    break
        
        return best_result
    
    def _predict_single(self, img_array):
        try:
            img_tensor = torch.FloatTensor(img_array).unsqueeze(0).to(self.device)
            
            with torch.no_grad():
                prediction = self.model(img_tensor)
                probabilities = torch.softmax(prediction, dim=1)
                confidence, predicted_class = torch.max(probabilities, 1)
            
            predicted_letter = self.label_encoder.inverse_transform([predicted_class.item()])[0]
            
            top_5_confidences, top_5_indices = torch.topk(probabilities[0], 5)
            top_5_letters = self.label_encoder.inverse_transform(top_5_indices.cpu().numpy())
            
            top_5 = [
                {'letter': letter, 'confidence': float(conf)}
                for letter, conf in zip(top_5_letters, top_5_confidences.cpu().numpy())
            ]
            
            return {
                'predicted_letter': predicted_letter,
                'confidence': float(confidence.item()),
                'top_5': top_5
            }
            
        except Exception as e:
            return None

# ============================================
# 9. FUNZIONI DI SUPPORTO
# ============================================
def load_single_image(args):
    img_path, letter, config, invert_colors = args
    img_array, _, _ = enhanced_preprocessing_with_rotation(img_path, invert_colors)
    if img_array is not None:
        return img_array, letter
    return None

def load_dataset_parallel(dataset_path, max_per_class=None, num_workers=8, invert_colors=True):
    images, labels = [], []
    
    if not os.path.exists(dataset_path):
        raise FileNotFoundError(f"Directory '{dataset_path}' non trovata!")
    
    letters = sorted([d for d in os.listdir(dataset_path) 
                     if os.path.isdir(os.path.join(dataset_path, d))])
    
    for letter in letters:
        letter_path = os.path.join(dataset_path, letter)
        image_files = [f for f in os.listdir(letter_path) 
                      if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]
        
        if max_per_class and max_per_class > 0:
            image_files = image_files[:max_per_class]
        
        args_list = [(os.path.join(letter_path, img_file), letter, config, invert_colors) 
                    for img_file in image_files]
        
        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            future_to_args = {executor.submit(load_single_image, args): args 
                            for args in args_list}
            
            for future in as_completed(future_to_args):
                try:
                    result = future.result(timeout=10)
                    if result is not None:
                        img_array, lbl = result
                        images.append(img_array)
                        labels.append(lbl)
                except Exception:
                    pass
    
    return np.array(images), np.array(labels)

def load_dataset(dataset_path):
    return load_dataset_parallel(
        dataset_path, 
        max_per_class=config.MAX_IMAGES_PER_CLASS if config.MAX_IMAGES_PER_CLASS > 0 else None,
        num_workers=config.NUM_WORKERS,
        invert_colors=config.INVERT_COLORS
    )

def prepare_data(images, labels):
    label_encoder = LabelEncoder()
    encoded_labels = label_encoder.fit_transform(labels)
    
    np.save(config.LABEL_ENCODER_PATH, label_encoder.classes_)
    
    X_temp, X_test, y_temp, y_test = train_test_split(
        images, encoded_labels, 
        test_size=config.TEST_SPLIT, 
        stratify=encoded_labels,
        random_state=42
    )
    
    X_train, X_val, y_train, y_val = train_test_split(
        X_temp, y_temp, 
        test_size=config.VALIDATION_SPLIT/(1-config.TEST_SPLIT), 
        stratify=y_temp,
        random_state=42
    )
    
    print(f"\nDistribuzione dati:")
    print(f"Training: {X_train.shape[0]} immagini")
    print(f"Validation: {X_val.shape[0]} immagini")
    print(f"Test: {X_test.shape[0]} immagini")
    
    return X_train, X_val, X_test, y_train, y_val, y_test, label_encoder

def evaluate_model(model, test_loader, label_encoder):
    model.eval()
    all_predictions, all_targets = [], []
    
    with torch.no_grad():
        for inputs, targets in test_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            outputs = model(inputs)
            _, predicted = outputs.max(1)
            all_predictions.extend(predicted.cpu().numpy())
            all_targets.extend(targets.cpu().numpy())
    
    all_predictions_letters = label_encoder.inverse_transform(all_predictions)
    all_targets_letters = label_encoder.inverse_transform(all_targets)
    
    print("\n📊 REPORT CLASSIFICAZIONE:")
    print(classification_report(all_targets_letters, all_predictions_letters))
    
    cm = confusion_matrix(all_targets_letters, all_predictions_letters)
    accuracy = np.sum(np.diag(cm)) / np.sum(cm)
    print(f"📈 Accuratezza: {accuracy:.4f} ({accuracy*100:.2f}%)")

# ============================================
# 10. FUNZIONE PRINCIPALE
# ============================================
def main_rotated():
    print("\n" + "="*70)
    print("OCR - RICONOSCIMENTO LETTERE CON ROTAZIONI (±45°)")
    print("="*70)
    
    device = check_system()
    monitor_gpu = setup_gpu_memory(config.GPU_MEMORY_LIMIT)
    
    # Carica dataset
    print("\n📥 CARICAMENTO DATASET...")
    images, labels = load_dataset(config.DATASET_PATH)
    
    if len(images) == 0:
        print("❌ Nessuna immagine caricata.")
        return
    
    # Prepara dati
    print("\n🔧 PREPARAZIONE DATI...")
    X_train, X_val, X_test, y_train, y_val, y_test, label_encoder = prepare_data(images, labels)
    
    # Crea dataset
    print("\n🔄 CREAZIONE DATASET CON AUGMENTATION...")
    train_dataset = AugmentedLetterDataset(X_train, y_train, augment=True)
    val_dataset = AugmentedLetterDataset(X_val, y_val, augment=False)
    test_dataset = AugmentedLetterDataset(X_test, y_test, augment=False)
    
    train_loader = DataLoader(train_dataset, batch_size=config.BATCH_SIZE, 
                            shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_dataset, batch_size=config.BATCH_SIZE, 
                          shuffle=False, num_workers=4, pin_memory=True)
    test_loader = DataLoader(test_dataset, batch_size=config.BATCH_SIZE, 
                           shuffle=False, num_workers=4, pin_memory=True)
    
    # Crea modello
    print("\n🧠 CREAZIONE MODELLO ROBUSTO ALLE ROTAZIONI...")
    input_shape = (config.CHANNELS, config.IMG_HEIGHT, config.IMG_WIDTH)
    model = RotationRobustCNN(input_shape, config.NUM_CLASSES)
    model.to(device)
    
    print(f"  Input shape: {input_shape}")
    print(f"  Parametri totali: {sum(p.numel() for p in model.parameters()):,}")
    
    # Training
    print("\n🚀 INIZIO TRAINING AVANZATO...")
    history, trained_model = train_with_rotation_testing(
        model, train_loader, val_loader, label_encoder, monitor_gpu
    )
    
    # Salva modello
    print("\n💾 SALVATAGGIO MODELLO...")
    torch.save(trained_model.state_dict(), config.MODEL_SAVE_PATH)
    print(f"  Modello salvato in: {config.MODEL_SAVE_PATH}")
    
    # Valutazione
    print("\n📈 VALUTAZIONE...")
    evaluate_model(trained_model, test_loader, label_encoder)
    
    print("\n" + "="*70)
    print("TRAINING COMPLETATO CON SUCCESSO! 🎉")
    print("="*70)

# ============================================
# ESECUZIONE
# ============================================
if __name__ == "__main__":
    try:
        main_rotated()
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrotto dall'utente")
    except Exception as e:
        print(f"\n❌ Errore: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if torch.cuda.is_available():
            torch.cuda.empty_cache()