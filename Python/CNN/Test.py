import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageEnhance, ImageOps, ImageFilter
import torch
import torch.nn as nn
from sklearn.preprocessing import LabelEncoder
import warnings
import sys
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk, scrolledtext
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import json
from datetime import datetime
from scipy.ndimage import rotate
import time
import csv

warnings.filterwarnings('ignore')

# ============================================
# CONFIGURAZIONE
# ============================================
class TestConfig:
    # PARAMETRI DEL MODELLO (DEVONO ESSERE UGUALI AL TRAINING)
    IMG_HEIGHT = 64
    IMG_WIDTH = 64
    CHANNELS = 1
    NUM_CLASSES = 26
    INVERT_COLORS = False
    
    # PERCORSI FILE (MODIFICABILI)
    MODEL_PATH = "letter_recognition_model_rotated.pth"
    BEST_MODEL_PATH = "letter_recognition_model_rotated_best.pth"
    LABEL_ENCODER_PATH = "label_encoder_rotated.npy"
    HISTORY_PATH = "training_history.json"
    
    # PARAMETRI RICONOSCIMENTO (CONFIGURABILI)
    USE_MULTI_ORIENTATION = True
    # MODIFICA: solo angoli da -45° a +45°
    ORIENTATION_ANGLES = [-45, -30, -15, 0, 15, 30, 45]
    MIN_CONFIDENCE_THRESHOLD = 0.6  # Soglia minima di confidenza
    USE_CONSENSUS_VOTING = True  # Usa voto di maggioranza tra orientamenti
    CONSENSUS_MIN_VOTES = 3  # Minimo orientamenti concordanti
    
    # CORREZIONI CONFUSIONI (CONFIGURABILI)
    USE_CONFUSION_CORRECTION = True
    CONFUSION_MAP = {
        'W': 'A', 'A': 'W',
        'M': 'W', 'V': 'A',
        'O': 'Q', 'Q': 'O',
        'I': 'L', 'L': 'I',
        'S': 'Z', 'Z': 'S'
    }
    CONFUSION_THRESHOLD = 0.15  # Differenza massima per correggere
    
    # TEST DI ROBUSTEZZA (CONFIGURABILI)
    TEST_ROTATION_ANGLES = list(range(-45, 46, 15))  # [-45, -30, ..., 0, ..., 45]
    
    # VISUALIZZAZIONE (CONFIGURABILI)
    SHOW_TOP_PREDICTIONS = 5
    SHOW_CONFIDENCE_BARS = True
    SHOW_PREPROCESSED_IMAGE = True
    
    # TEST BATCH (CONFIGURABILI)
    BATCH_PROCESSING = True
    BATCH_SIZE = 32
    SAVE_RESULTS_CSV = True
    SAVE_RESULTS_JSON = True
    
    # INTERFACCIA (CONFIGURABILI)
    GUI_THEME = "clam"  # 'clam', 'alt', 'default', 'classic'
    GUI_FONT = ("Arial", 10)
    GUI_TITLE = "OCR - Test Modello Lettere Rotate"
    
    # DEBUG (CONFIGURABILI)
    VERBOSE_LEVEL = 2  # 0: minimo, 1: medio, 2: dettagliato, 3: debug
    SAVE_TEMP_IMAGES = False
    LOG_TO_FILE = True

config = TestConfig()

# ============================================
# CLASSE PER PREPROCESSING
# ============================================
class ImagePreprocessor:
    """Preprocessing identico al training"""
    
    @staticmethod
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
            if config.VERBOSE_LEVEL >= 2:
                print(f"[ERROR] Preprocessing fallito: {e}")
            return None, None, 0

# ============================================
# ARCHITETTURA DEL MODELLO
# ============================================
class RotationRobustCNN(nn.Module):
    """Architettura identica al training"""
    def __init__(self, input_shape, num_classes, dropout_rate=0.5):
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
            nn.AdaptiveAvgPool2d((4, 4)),
            nn.Dropout2d(0.3),
        )
        
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(256 * 4 * 4, 512),
            nn.ReLU(inplace=True),
            nn.BatchNorm1d(512),
            nn.Dropout(dropout_rate),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.BatchNorm1d(256),
            nn.Dropout(dropout_rate),
            nn.Linear(256, num_classes),
        )
    
    def forward(self, x):
        x = self.block1(x)
        x = self.block2(x)
        x = self.block3(x)
        x = self.classifier(x)
        return x

# ============================================
# CLASSE PRINCIPALE PER RICONOSCIMENTO
# ============================================
class AdvancedLetterRecognizer:
    """Riconoscitore avanzato con supporto rotazioni"""
    
    def __init__(self, model_path=None, encoder_path=None, use_best_model=False):
        # Setup dispositivo
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Carica configurazione
        self._load_config()
        
        # Determina percorso modello
        if model_path:
            self.model_path = model_path
        elif use_best_model and os.path.exists(config.BEST_MODEL_PATH):
            self.model_path = config.BEST_MODEL_PATH
        else:
            self.model_path = config.MODEL_PATH
        
        # Carica label encoder
        self.label_encoder = self._load_label_encoder(encoder_path)
        
        # Carica modello
        self.model = self._load_model()
        
        # Inizializza preprocessor
        self.preprocessor = ImagePreprocessor()
        
        # Statistiche
        self.stats = {
            'total_predictions': 0,
            'successful_predictions': 0,
            'failed_predictions': 0,
            'avg_confidence': 0.0,
            'total_time': 0.0,
            'predictions_by_letter': {},
            'confusion_matrix': {}
        }
        
        self._print_init_info()
    
    def _load_config(self):
        """Carica configurazione dal file history se esiste"""
        try:
            if os.path.exists(config.HISTORY_PATH):
                with open(config.HISTORY_PATH, 'r') as f:
                    history = json.load(f)
                    if 'config' in history:
                        training_config = history['config']
                        # Aggiorna configurazione con parametri training
                        config.IMG_HEIGHT = training_config.get('IMG_HEIGHT', config.IMG_HEIGHT)
                        config.IMG_WIDTH = training_config.get('IMG_WIDTH', config.IMG_WIDTH)
                        config.CHANNELS = training_config.get('CHANNELS', config.CHANNELS)
                        config.INVERT_COLORS = training_config.get('INVERT_COLORS', config.INVERT_COLORS)
                        
                        if config.VERBOSE_LEVEL >= 1:
                            print(f"✅ Configurazione caricata da {config.HISTORY_PATH}")
        except Exception as e:
            if config.VERBOSE_LEVEL >= 2:
                print(f"⚠️  Impossibile caricare configurazione: {e}")
    
    def _load_label_encoder(self, encoder_path=None):
        """Carica label encoder"""
        try:
            label_encoder = LabelEncoder()
            path_to_load = encoder_path or config.LABEL_ENCODER_PATH
            label_encoder.classes_ = np.load(path_to_load, allow_pickle=True)
            return label_encoder
        except Exception as e:
            raise FileNotFoundError(f"❌ Impossibile caricare label encoder: {e}")
    
    def _load_model(self):
        """Carica modello addestrato"""
        try:
            input_shape = (config.CHANNELS, config.IMG_HEIGHT, config.IMG_WIDTH)
            model = RotationRobustCNN(input_shape, config.NUM_CLASSES)
            
            if config.VERBOSE_LEVEL >= 1:
                print(f"📦 Caricamento modello da: {self.model_path}")
            
            if torch.cuda.is_available():
                model.load_state_dict(torch.load(self.model_path))
            else:
                model.load_state_dict(torch.load(self.model_path, 
                                                map_location=torch.device('cpu')))
            
            model.to(self.device)
            model.eval()
            
            if config.VERBOSE_LEVEL >= 1:
                print(f"✅ Modello caricato correttamente")
                print(f"   Classi: {list(self.label_encoder.classes_)}")
                print(f"   Parametri: {sum(p.numel() for p in model.parameters()):,}")
                print(f"   Dispositivo: {self.device}")
            
            return model
        except Exception as e:
            raise FileNotFoundError(f"❌ Impossibile caricare modello: {e}")
    
    def _print_init_info(self):
        """Stampa info inizializzazione"""
        print("\n" + "="*60)
        print("🔄 INIZIALIZZAZIONE RICONOSCITORE")
        print("="*60)
        print(f"📁 Modello: {os.path.basename(self.model_path)}")
        print(f"🎯 Classi: {len(self.label_encoder.classes_)} lettere")
        print(f"🖼️  Dimensione input: {config.IMG_WIDTH}x{config.IMG_HEIGHT}")
        print(f"🔄 Multi-orientamento: {'ATTIVO' if config.USE_MULTI_ORIENTATION else 'DISATTIVO'}")
        if config.USE_MULTI_ORIENTATION:
            print(f"   Angoli testati: {config.ORIENTATION_ANGLES}")
        print(f"🔧 Correzione confusioni: {'ATTIVO' if config.USE_CONFUSION_CORRECTION else 'DISATTIVO'}")
        print(f"📊 Soglia confidenza: {config.MIN_CONFIDENCE_THRESHOLD}")
        print("="*60)
    
    def predict_single_image(self, image_path):
        """Predice una singola immagine"""
        start_time = time.time()
        filename = os.path.basename(image_path)
        
        if config.VERBOSE_LEVEL >= 2:
            print(f"\n🔍 Analisi: {filename}")
        
        try:
            if config.USE_MULTI_ORIENTATION:
                result = self._predict_multi_orientation(image_path)
            else:
                result = self._predict_single_orientation(image_path)
            
            if result and 'error' not in result:
                # Aggiorna statistiche
                self._update_stats(result, time.time() - start_time)
                
                if config.VERBOSE_LEVEL >= 1:
                    self._print_prediction_result(result, filename)
                
                return result
            else:
                error_msg = result.get('error', 'Predizione fallita') if result else 'Predizione fallita'
                if config.VERBOSE_LEVEL >= 1:
                    print(f"❌ {filename}: {error_msg}")
                
                self.stats['failed_predictions'] += 1
                return {'error': error_msg, 'image_path': image_path}
                
        except Exception as e:
            error_msg = f"Errore durante la predizione: {str(e)}"
            if config.VERBOSE_LEVEL >= 1:
                print(f"❌ {filename}: {error_msg}")
            
            self.stats['failed_predictions'] += 1
            return {'error': error_msg, 'image_path': image_path}
    
    def _predict_multi_orientation(self, image_path):
        """Predice testando multiple orientazioni"""
        results_by_orientation = []
        
        # MODIFICA: Filtra angoli entro ±45°
        valid_angles = [a for a in config.ORIENTATION_ANGLES if abs(a) <= 45]
        
        if config.VERBOSE_LEVEL >= 2:
            print(f"  Angoli validi: {valid_angles}")
        
        for angle in valid_angles:  # MODIFICA: usa valid_angles
            try:
                # Preprocess con rotazione
                img_array, original_size, _ = self.preprocessor.enhanced_preprocessing_with_rotation(
                    image_path, 
                    invert_colors=config.INVERT_COLORS,
                    test_rotation=True,
                    angle=angle
                )
                
                if img_array is None:
                    continue
                
                # Predici
                prediction = self._predict_tensor(img_array)
                if prediction:
                    prediction['angle'] = angle
                    prediction['original_size'] = original_size
                    results_by_orientation.append(prediction)
                    
            except Exception as e:
                if config.VERBOSE_LEVEL >= 3:
                    print(f"  ⚠️  Errore angolo {angle}°: {e}")
                continue
        
        if not results_by_orientation:
            return {'error': 'Nessuna predizione valida per qualsiasi orientamento'}
        
        # Applica voto di maggioranza se richiesto
        if config.USE_CONSENSUS_VOTING:
            final_result = self._apply_consensus_voting(results_by_orientation)
        else:
            # Prendi il risultato con confidenza più alta
            final_result = max(results_by_orientation, key=lambda x: x['confidence'])
        
        # Aggiungi informazioni extra
        final_result['all_orientations'] = [
            {'angle': r['angle'], 'letter': r['predicted_letter'], 'confidence': r['confidence']}
            for r in results_by_orientation
        ]
        
        # Applica correzione confusioni
        if config.USE_CONFUSION_CORRECTION:
            final_result = self._apply_confusion_correction(final_result)
        
        return final_result
    
    def _predict_single_orientation(self, image_path):
        """Predice con orientamento normale"""
        img_array, original_size, _ = self.preprocessor.enhanced_preprocessing_with_rotation(
            image_path, 
            invert_colors=config.INVERT_COLORS
        )
        
        if img_array is None:
            return {'error': 'Preprocessing fallito'}
        
        prediction = self._predict_tensor(img_array)
        if not prediction:
            return {'error': 'Predizione fallita'}
        
        prediction['original_size'] = original_size
        
        # Applica correzione confusioni
        if config.USE_CONFUSION_CORRECTION:
            prediction = self._apply_confusion_correction(prediction)
        
        return prediction
    
    def _predict_tensor(self, img_array):
        """Predice da tensore"""
        try:
            # Verifica dimensioni
            expected_shape = (config.CHANNELS, config.IMG_HEIGHT, config.IMG_WIDTH)
            if img_array.shape != expected_shape:
                if config.VERBOSE_LEVEL >= 3:
                    print(f"  ⚠️  Shape mismatch: {img_array.shape} != {expected_shape}")
                return None
            
            # Converti in tensore
            img_tensor = torch.FloatTensor(img_array).unsqueeze(0).to(self.device)
            
            # Predici
            with torch.no_grad():
                prediction = self.model(img_tensor)
                probabilities = torch.softmax(prediction, dim=1)
                confidence, predicted_class = torch.max(probabilities, 1)
            
            predicted_letter = self.label_encoder.inverse_transform([predicted_class.item()])[0]
            
            # Top predictions
            top_k = min(config.SHOW_TOP_PREDICTIONS, config.NUM_CLASSES)
            top_confidences, top_indices = torch.topk(probabilities[0], top_k)
            top_letters = self.label_encoder.inverse_transform(top_indices.cpu().numpy())
            
            top_predictions = [
                {"letter": letter, "confidence": float(conf)}
                for letter, conf in zip(top_letters, top_confidences.cpu().numpy())
            ]
            
            # Verifica soglia minima
            if float(confidence.item()) < config.MIN_CONFIDENCE_THRESHOLD:
                if config.VERBOSE_LEVEL >= 2:
                    print(f"  ⚠️  Confidenza sotto soglia: {float(confidence.item()):.3f} < {config.MIN_CONFIDENCE_THRESHOLD}")
            
            return {
                "predicted_letter": predicted_letter,
                "confidence": float(confidence.item()),
                "top_predictions": top_predictions,
                "all_probabilities": probabilities[0].cpu().numpy().tolist(),
                "timestamp": datetime.now().isoformat()
            }
            
        except Exception as e:
            if config.VERBOSE_LEVEL >= 3:
                print(f"  ❌ Errore predizione tensore: {e}")
            return None
    
    def _apply_consensus_voting(self, results_by_orientation):
        """Applica voto di maggioranza tra orientamenti"""
        letter_votes = {}
        
        for result in results_by_orientation:
            letter = result['predicted_letter']
            confidence = result['confidence']
            
            if letter not in letter_votes:
                letter_votes[letter] = {
                    'count': 0,
                    'total_confidence': 0.0,
                    'angles': [],
                    'best_confidence': 0.0
                }
            
            letter_votes[letter]['count'] += 1
            letter_votes[letter]['total_confidence'] += confidence
            letter_votes[letter]['angles'].append(result['angle'])
            letter_votes[letter]['best_confidence'] = max(letter_votes[letter]['best_confidence'], confidence)
        
        # Trova lettera con più voti
        best_letter = None
        best_votes = 0
        best_avg_confidence = 0.0
        
        for letter, votes in letter_votes.items():
            if votes['count'] > best_votes:
                best_votes = votes['count']
                best_letter = letter
                best_avg_confidence = votes['total_confidence'] / votes['count']
            elif votes['count'] == best_votes:
                # In caso di parità, usa confidenza media più alta
                avg_conf = votes['total_confidence'] / votes['count']
                if avg_conf > best_avg_confidence:
                    best_letter = letter
                    best_avg_confidence = avg_conf
        
        # Verifica se abbiamo consenso sufficiente
        if best_votes >= config.CONSENSUS_MIN_VOTES:
            # Prendi il risultato originale con confidenza più alta per quella lettera
            best_result = None
            for result in results_by_orientation:
                if result['predicted_letter'] == best_letter:
                    if best_result is None or result['confidence'] > best_result['confidence']:
                        best_result = result
            
            if best_result:
                # Aggiorna con confidenza media
                best_result['confidence'] = best_avg_confidence
                best_result['consensus_votes'] = best_votes
                best_result['consensus_letter'] = best_letter
                
                if config.VERBOSE_LEVEL >= 2:
                    print(f"  ✅ Consenso raggiunto: {best_letter} ({best_votes}/{len(results_by_orientation)} voti)")
                
                return best_result
        
        # Se nessun consenso, usa il risultato con confidenza più alta
        return max(results_by_orientation, key=lambda x: x['confidence'])
    
    def _apply_confusion_correction(self, result):
        """Corregge confusioni comuni tra lettere"""
        predicted_letter = result['predicted_letter']
        
        if predicted_letter in config.CONFUSION_MAP:
            expected_confusion = config.CONFUSION_MAP[predicted_letter]
            
            # Cerca la lettera confusa nelle top predictions
            for pred in result['top_predictions']:
                if pred['letter'] == expected_confusion:
                    # Calcola differenza di confidenza
                    diff = abs(result['top_predictions'][0]['confidence'] - pred['confidence'])
                    
                    if diff < config.CONFUSION_THRESHOLD:
                        if config.VERBOSE_LEVEL >= 2:
                            print(f"  🔄 Correzione confusione: {predicted_letter} → {expected_confusion}")
                        
                        result['predicted_letter'] = expected_confusion
                        result['original_letter'] = predicted_letter
                        result['confusion_corrected'] = True
                        result['confidence_diff'] = diff
                        break
        
        return result
    
    def _update_stats(self, result, prediction_time):
        """Aggiorna statistiche"""
        self.stats['total_predictions'] += 1
        self.stats['successful_predictions'] += 1
        self.stats['total_time'] += prediction_time
        
        # Aggiorna confidenza media
        old_total = self.stats['successful_predictions'] - 1
        old_avg = self.stats['avg_confidence']
        self.stats['avg_confidence'] = (old_avg * old_total + result['confidence']) / self.stats['successful_predictions']
        
        # Aggiorna conteggio per lettera
        letter = result['predicted_letter']
        if letter not in self.stats['predictions_by_letter']:
            self.stats['predictions_by_letter'][letter] = 0
        self.stats['predictions_by_letter'][letter] += 1
    
    def _print_prediction_result(self, result, filename):
        """Stampa risultato predizione"""
        letter = result['predicted_letter']
        confidence = result['confidence']
        
        if config.VERBOSE_LEVEL >= 2:
            print(f"  ✅ {filename}: '{letter}' ({confidence:.2%})")
            
            if 'consensus_votes' in result:
                print(f"     Consenso: {result['consensus_votes']} voti")
            
            if 'confusion_corrected' in result and result['confusion_corrected']:
                print(f"     Correzione: {result['original_letter']} → {letter}")
            
            if config.VERBOSE_LEVEL >= 3 and 'all_orientations' in result:
                print(f"     Tutti gli orientamenti:")
                for orient in result['all_orientations']:
                    print(f"       {orient['angle']:3d}°: {orient['letter']} ({orient['confidence']:.2%})")
        
        elif config.VERBOSE_LEVEL >= 1:
            print(f"  ✅ {filename}: '{letter}' ({confidence:.1%})")
    
    def predict_batch(self, image_paths):
        """Predice un batch di immagini"""
        if config.VERBOSE_LEVEL >= 1:
            print(f"\n{'='*60}")
            print(f"📦 PROCESSING BATCH: {len(image_paths)} immagini")
            print(f"{'='*60}")
        
        start_time = time.time()
        results = []
        
        for i, img_path in enumerate(image_paths):
            if config.VERBOSE_LEVEL >= 2:
                print(f"\n[{i+1}/{len(image_paths)}] ", end="")
            
            result = self.predict_single_image(img_path)
            results.append({
                'image_path': img_path,
                'result': result
            })
        
        total_time = time.time() - start_time
        
        if config.VERBOSE_LEVEL >= 1:
            self._print_batch_summary(total_time, len(image_paths))
        
        return results
    
    def _print_batch_summary(self, total_time, num_images):
        """Stampa riepilogo batch"""
        print(f"\n{'='*60}")
        print(f"📊 RIEPILOGO BATCH")
        print(f"{'='*60}")
        print(f"Immagini processate: {num_images}")
        print(f"Successo: {self.stats['successful_predictions']}")
        print(f"Falliti: {self.stats['failed_predictions']}")
        print(f"Tempo totale: {total_time:.2f}s")
        print(f"Tempo medio per immagine: {total_time/num_images:.2f}s")
        print(f"Confidenza media: {self.stats['avg_confidence']:.2%}")
        
        if self.stats['predictions_by_letter']:
            print(f"\n📈 Distribuzione lettere:")
            for letter, count in sorted(self.stats['predictions_by_letter'].items(), 
                                      key=lambda x: x[1], reverse=True)[:10]:
                print(f"  {letter}: {count}")
        
        print(f"{'='*60}")
    
    def test_rotation_robustness(self, image_path, test_angles=None):
        """Testa robustezza a diverse rotazioni"""
        # MODIFICA: usa TEST_ROTATION_ANGLES di default
        if test_angles is None:
            test_angles = config.TEST_ROTATION_ANGLES
        
        # MODIFICA: Filtra angoli entro ±45°
        valid_angles = [a for a in test_angles if abs(a) <= 45]
        
        print(f"\n🧪 TEST ROBUSTEZZA ROTAZIONI")
        print(f"📁 Immagine: {os.path.basename(image_path)}")
        print(f"📐 Angoli testati: {valid_angles}")
        
        results = []
        
        for angle in valid_angles:  # MODIFICA: usa valid_angles
            try:
                # Preprocess con rotazione specifica
                img_array, _, _ = self.preprocessor.enhanced_preprocessing_with_rotation(
                    image_path,
                    invert_colors=config.INVERT_COLORS,
                    test_rotation=True,
                    angle=angle
                )
                
                if img_array is None:
                    continue
                
                # Predici
                prediction = self._predict_tensor(img_array)
                if prediction:
                    results.append({
                        'angle': angle,
                        'letter': prediction['predicted_letter'],
                        'confidence': prediction['confidence'],
                        'top_3': prediction['top_predictions'][:3]
                    })
                    
                    print(f"  {angle:3d}°: {prediction['predicted_letter']} ({prediction['confidence']:.2%})")
                    
            except Exception as e:
                print(f"  ❌ {angle}°: Errore - {e}")
        
        return results
    
    def export_results(self, results, output_format='json'):
        """Esporta risultati in vari formati"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if output_format.lower() == 'json' and config.SAVE_RESULTS_JSON:
            output_file = f"predictions_{timestamp}.json"
            export_data = {
                'export_timestamp': datetime.now().isoformat(),
                'model_used': os.path.basename(self.model_path),
                'config': {k: v for k, v in vars(config).items() if not k.startswith('_')},
                'statistics': self.stats,
                'results': results
            }
            
            with open(output_file, 'w') as f:
                json.dump(export_data, f, indent=2, default=str)
            
            print(f"💾 Risultati JSON salvati in: {output_file}")
            return output_file
        
        elif output_format.lower() == 'csv' and config.SAVE_RESULTS_CSV:
            output_file = f"predictions_{timestamp}.csv"
            
            with open(output_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['Image', 'Predicted_Letter', 'Confidence', 'Top1', 'Top2', 'Top3', 
                               'Consensus_Votes', 'Corrected', 'Timestamp'])
                
                for item in results:
                    result = item['result']
                    if 'error' not in result:
                        writer.writerow([
                            os.path.basename(item['image_path']),
                            result.get('predicted_letter', 'ERROR'),
                            result.get('confidence', 0.0),
                            result['top_predictions'][0]['letter'] if 'top_predictions' in result else '',
                            result['top_predictions'][1]['letter'] if 'top_predictions' in result and len(result['top_predictions']) > 1 else '',
                            result['top_predictions'][2]['letter'] if 'top_predictions' in result and len(result['top_predictions']) > 2 else '',
                            result.get('consensus_votes', ''),
                            'YES' if result.get('confusion_corrected', False) else 'NO',
                            result.get('timestamp', '')
                        ])
            
            print(f"💾 Risultati CSV salvati in: {output_file}")
            return output_file
        
        return None

# ============================================
# INTERFACCIA GRAFICA
# ============================================
class LetterRecognitionGUI:
    def __init__(self, root):
        self.root = root
        self.root.title(config.GUI_TITLE)
        self.root.geometry("1400x900")
        
        # Inizializza riconoscitore
        self.recognizer = None
        self.current_image_path = None
        self.current_result = None
        self.batch_results = []
        
        # Setup stili
        self.setup_styles()
        
        # Crea interfaccia
        self.create_widgets()
        
        # Carica modello all'avvio
        self.load_model()
    
    def setup_styles(self):
        """Configura stili per l'interfaccia"""
        style = ttk.Style()
        style.theme_use(config.GUI_THEME)
        
        # Colori personalizzati
        self.colors = {
            'bg': "#f5f5f5",
            'primary': "#2c3e50",
            'secondary': "#34495e",
            'accent': "#3498db",
            'success': "#27ae60",
            'warning': "#f39c12",
            'danger': "#e74c3c",
            'light': "#ecf0f1",
            'dark': "#2c3e50"
        }
        
        self.root.configure(bg=self.colors['bg'])
        
        # Configura stili ttk
        style.configure('Title.TLabel', font=("Arial", 18, "bold"), 
                       foreground=self.colors['primary'])
        style.configure('Section.TLabel', font=("Arial", 14, "bold"),
                       foreground=self.colors['secondary'])
        style.configure('Result.TLabel', font=("Arial", 48, "bold"),
                       foreground=self.colors['accent'])
    
    def create_widgets(self):
        """Crea tutti i widget dell'interfaccia"""
        
        # Frame principale
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Frame superiore (titolo e controlli)
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill="x", pady=(0, 10))
        
        # Titolo
        title_label = ttk.Label(top_frame, text=config.GUI_TITLE, 
                               style='Title.TLabel')
        title_label.pack(side="left")
        
        # Controlli modello
        model_frame = ttk.Frame(top_frame)
        model_frame.pack(side="right")
        
        ttk.Button(model_frame, text="🔄 Ricarica Modello", 
                  command=self.reload_model).pack(side="left", padx=5)
        ttk.Button(model_frame, text="⚙️  Configura", 
                  command=self.open_config).pack(side="left", padx=5)
        
        # Frame centrale
        center_frame = ttk.Frame(main_frame)
        center_frame.pack(fill="both", expand=True)
        
        # Frame sinistro (immagine)
        left_frame = ttk.LabelFrame(center_frame, text="IMMAGINE", 
                                   padding=10)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))
        
        # Frame destro (risultati)
        right_frame = ttk.LabelFrame(center_frame, text="RISULTATI", 
                                    padding=10)
        right_frame.pack(side="right", fill="both", expand=True, padx=(10, 0))
        
        # Widget immagine
        self.create_image_widgets(left_frame)
        
        # Widget risultati
        self.create_result_widgets(right_frame)
        
        # Frame inferiore (controlli e log)
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill="x", pady=(10, 0))
        
        self.create_control_widgets(bottom_frame)
        self.create_log_widgets(bottom_frame)
    
    def create_image_widgets(self, parent):
        """Crea widget per visualizzazione immagine"""
        
        # Canvas per immagine
        self.image_canvas = tk.Canvas(parent, bg="white", 
                                     highlightthickness=1, 
                                     highlightbackground="#ddd")
        self.image_canvas.pack(fill="both", expand=True)
        
        # Info immagine
        self.image_info = ttk.Label(parent, text="Nessuna immagine caricata")
        self.image_info.pack(pady=(10, 0))
    
    def create_result_widgets(self, parent):
        """Crea widget per visualizzazione risultati"""
        
        # Lettera predetta
        result_frame = ttk.Frame(parent)
        result_frame.pack(pady=20)
        
        self.letter_label = ttk.Label(result_frame, text="?", 
                                     style='Result.TLabel')
        self.letter_label.pack()
        
        # Confidenza
        self.confidence_label = ttk.Label(result_frame, 
                                         text="Confidenza: --%",
                                         font=("Arial", 14))
        self.confidence_label.pack(pady=10)
        
        # Separatore
        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=20)
        
        # Top predictions
        top_pred_frame = ttk.LabelFrame(parent, text="TOP PREDIZIONI", 
                                       padding=10)
        top_pred_frame.pack(fill="both", expand=True)
        
        # Treeview per top predictions
        columns = ('Rank', 'Letter', 'Confidence')
        self.top_pred_tree = ttk.Treeview(top_pred_frame, columns=columns, 
                                         show='headings', height=5)
        
        for col in columns:
            self.top_pred_tree.heading(col, text=col)
            self.top_pred_tree.column(col, width=100, anchor='center')
        
        self.top_pred_tree.pack(fill="both", expand=True)
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(top_pred_frame, 
                                 orient="vertical", 
                                 command=self.top_pred_tree.yview)
        self.top_pred_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        
        # Dettagli
        details_frame = ttk.LabelFrame(parent, text="DETTAGLI", padding=10)
        details_frame.pack(fill="x", pady=(10, 0))
        
        self.details_text = tk.Text(details_frame, height=6, font=("Courier", 10),
                                   bg=self.colors['light'], relief="flat")
        self.details_text.pack(fill="both", expand=True)
    
    def create_control_widgets(self, parent):
        """Crea widget di controllo"""
    
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill="x", pady=(0, 10))
        
        # Bottoni principali
        buttons = [
            ("📁 Carica Immagine", self.load_image, self.colors['primary']),
            ("📂 Carica Cartella", self.load_folder, self.colors['secondary']),
            ("🔍 Riconosci", self.recognize_image, self.colors['accent']),
            ("🧪 Test Rotazioni", self.test_rotations, self.colors['warning']),
            ("💾 Esporta Risultati", self.export_results, self.colors['success']),
            ("📊 Statistiche", self.show_stats, self.colors['dark'])
        ]
        
        for text, command, color in buttons:
            # Crea nome univoco per il bottone
            if "Carica Immagine" in text:
                btn_name = "btn_carica"
            elif "Carica Cartella" in text:
                btn_name = "btn_carica_cartella"
            elif "Riconosci" in text:
                btn_name = "btn_riconosci"
            elif "Test Rotazioni" in text:
                btn_name = "btn_test"
            elif "Esporta Risultati" in text:
                btn_name = "btn_esporta"
            elif "Statistiche" in text:
                btn_name = "btn_statistiche"
            else:
                continue
            
            btn = tk.Button(control_frame, text=text, command=command,
                        font=("Arial", 11), bg=color, fg="white",
                        padx=15, pady=8, relief="raised", bd=2)
            btn.pack(side="left", padx=5)
            
            # Imposta come attributo della classe
            setattr(self, btn_name, btn)
        
        # Disabilita bottoni inizialmente
        self.btn_riconosci.config(state="disabled")
        self.btn_esporta.config(state="disabled")


    def create_log_widgets(self, parent):
        """Crea widget per log"""
        
        log_frame = ttk.LabelFrame(parent, text="LOG", padding=10)
        log_frame.pack(fill="both", expand=True)
        
        # ScrolledText per log
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8,
                                                 font=("Courier", 9),
                                                 bg="#2c3e50", fg="white")
        self.log_text.pack(fill="both", expand=True)
        
        # Pulsanti log
        log_buttons = ttk.Frame(log_frame)
        log_buttons.pack(fill="x", pady=(5, 0))
        
        ttk.Button(log_buttons, text="Pulisci Log", 
                  command=self.clear_log).pack(side="left", padx=5)
        ttk.Button(log_buttons, text="Salva Log", 
                  command=self.save_log).pack(side="left", padx=5)
    
    def load_model(self):
        """Carica il modello all'avvio"""
        try:
            self.log("🔧 Caricamento modello in corso...")
            self.recognizer = AdvancedLetterRecognizer()
            self.log("✅ Modello caricato con successo!")
            
            # Abilita controlli
            self.btn_riconosci.config(state="normal")
            
        except Exception as e:
            self.log(f"❌ Errore caricamento modello: {e}")
            messagebox.showerror("Errore", 
                               f"Impossibile caricare il modello:\n{str(e)}")
    
    def reload_model(self):
        """Ricarica il modello"""
        try:
            self.log("🔄 Ricaricamento modello...")
            self.recognizer = AdvancedLetterRecognizer()
            self.log("✅ Modello ricaricato!")
        except Exception as e:
            self.log(f"❌ Errore ricaricamento: {e}")
    
    def open_config(self):
        """Apre finestra configurazione"""
        config_window = tk.Toplevel(self.root)
        config_window.title("Configurazione")
        config_window.geometry("600x600")
        
        # Notebook per schede
        notebook = ttk.Notebook(config_window)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Scheda generale
        general_frame = ttk.Frame(notebook)
        notebook.add(general_frame, text="Generale")
        
        self.create_config_widgets(general_frame, [
            ("USE_MULTI_ORIENTATION", "Multi-orientamento", "checkbutton"),
            ("MIN_CONFIDENCE_THRESHOLD", "Soglia confidenza", "scale", 0.0, 1.0),
            ("USE_CONSENSUS_VOTING", "Voto di maggioranza", "checkbutton"),
            ("CONSENSUS_MIN_VOTES", "Voti minimi consenso", "spinbox", 1, 8)
        ])
        
        # AGGIUNTA: Widget per modificare angoli di rotazione
        angles_frame = ttk.LabelFrame(general_frame, text="Angoli di Rotazione", padding=10)
        angles_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(angles_frame, text="Angoli testati (da -45° a +45°):").pack(anchor="w")
        
        current_angles = ", ".join(str(a) for a in config.ORIENTATION_ANGLES)
        self.angles_var = tk.StringVar(value=current_angles)
        
        angles_entry = ttk.Entry(angles_frame, textvariable=self.angles_var, width=50)
        angles_entry.pack(fill="x", pady=5)
        
        ttk.Label(angles_frame, text="Esempio: -45, -30, -15, 0, 15, 30, 45", 
                 font=("Arial", 9), foreground="gray").pack(anchor="w")
        
        # Scheda correzioni
        correction_frame = ttk.Frame(notebook)
        notebook.add(correction_frame, text="Correzioni")
        
        self.create_config_widgets(correction_frame, [
            ("USE_CONFUSION_CORRECTION", "Correzione confusioni", "checkbutton"),
            ("CONFUSION_THRESHOLD", "Soglia correzione", "scale", 0.0, 0.5)
        ])
        
        # Scheda output
        output_frame = ttk.Frame(notebook)
        notebook.add(output_frame, text="Output")
        
        self.create_config_widgets(output_frame, [
            ("SAVE_RESULTS_CSV", "Salva CSV", "checkbutton"),
            ("SAVE_RESULTS_JSON", "Salva JSON", "checkbutton"),
            ("VERBOSE_LEVEL", "Livello verbose", "spinbox", 0, 3)
        ])
        
        # Pulsanti
        button_frame = ttk.Frame(config_window)
        button_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Button(button_frame, text="Applica", 
                  command=lambda: self.apply_config(config_window)).pack(side="right", padx=5)
        ttk.Button(button_frame, text="Annulla", 
                  command=config_window.destroy).pack(side="right", padx=5)
    
    def create_config_widgets(self, parent, config_items):
        """Crea widget configurazione"""
        for i, (attr, label, widget_type, *args) in enumerate(config_items):
            frame = ttk.Frame(parent)
            frame.pack(fill="x", padx=10, pady=5)
            
            ttk.Label(frame, text=label, width=25).pack(side="left")
            
            current_value = getattr(config, attr)
            
            if widget_type == "checkbutton":
                var = tk.BooleanVar(value=current_value)
                ttk.Checkbutton(frame, variable=var).pack(side="left")
                setattr(self, f"config_{attr}", var)
                
            elif widget_type == "scale":
                var = tk.DoubleVar(value=current_value)
                scale = ttk.Scale(frame, from_=args[0], to=args[1], 
                                variable=var, orient="horizontal")
                scale.pack(side="left", fill="x", expand=True, padx=10)
                value_label = ttk.Label(frame, text=f"{current_value:.2f}")
                value_label.pack(side="left", padx=5)
                
                # Aggiorna label quando cambia il valore
                def update_label(val, lbl=value_label):
                    lbl.config(text=f"{float(val):.2f}")
                
                var.trace("w", lambda *args: update_label(var.get()))
                setattr(self, f"config_{attr}", var)
                
            elif widget_type == "spinbox":
                var = tk.IntVar(value=current_value)
                spinbox = ttk.Spinbox(frame, from_=args[0], to=args[1], 
                                     textvariable=var, width=10)
                spinbox.pack(side="left")
                setattr(self, f"config_{attr}", var)
    
    def apply_config(self, window):
        """Applica configurazione modificata"""
        try:
            # Aggiorna configurazione
            for attr in dir(config):
                if not attr.startswith('_') and hasattr(self, f"config_{attr}"):
                    var = getattr(self, f"config_{attr}")
                    if isinstance(var, tk.BooleanVar):
                        setattr(config, attr, var.get())
                    elif isinstance(var, (tk.IntVar, tk.DoubleVar)):
                        setattr(config, attr, var.get())
            
            # MODIFICA: Aggiorna angoli di rotazione
            try:
                angles_str = self.angles_var.get()
                angles_list = [int(a.strip()) for a in angles_str.split(",") if a.strip()]
                
                # Verifica limiti
                for angle in angles_list:
                    if abs(angle) > 45:
                        raise ValueError(f"Angolo {angle}° fuori limite (±45°)")
                
                config.ORIENTATION_ANGLES = angles_list
                self.log(f"⚙️  Angoli aggiornati: {angles_list}")
                
            except ValueError as e:
                messagebox.showwarning("Attenzione", f"Angoli non validi: {e}\nUsati angoli predefiniti.")
                config.ORIENTATION_ANGLES = [-45, -30, -15, 0, 15, 30, 45]
            
            self.log("⚙️  Configurazione applicata")
            window.destroy()
            
        except Exception as e:
            self.log(f"❌ Errore applicazione configurazione: {e}")
    
    def load_image(self):
        """Carica una singola immagine"""
        filetypes = [
            ("Immagini", "*.png *.jpg *.jpeg *.bmp *.tiff"),
            ("Tutti i file", "*.*")
        ]
        
        file_path = filedialog.askopenfilename(title="Seleziona un'immagine",
                                              filetypes=filetypes)
        
        if file_path:
            self.current_image_path = file_path
            self.display_image(file_path)
            self.btn_riconosci.config(state="normal")
            self.btn_esporta.config(state="disabled")
            self.clear_results()
            self.log(f"📁 Immagine caricata: {os.path.basename(file_path)}")
    
    def load_folder(self):
        """Carica tutte le immagini da una cartella"""
        folder_path = filedialog.askdirectory(title="Seleziona una cartella")
        
        if folder_path:
            # Cerca tutte le immagini
            extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']
            image_files = []
            
            for ext in extensions:
                image_files.extend(Path(folder_path).glob(f'*{ext}'))
                image_files.extend(Path(folder_path).glob(f'*{ext.upper()}'))
            
            image_paths = [str(f) for f in image_files]
            
            if not image_paths:
                messagebox.showinfo("Nessuna immagine", 
                                  "Nessuna immagine trovata nella cartella.")
                return
            
            # Processa batch
            self.process_batch_images(image_paths)
    
    def display_image(self, image_path):
        """Visualizza l'immagine nel canvas"""
        try:
            # Carica immagine
            img = Image.open(image_path)
            
            # Calcola dimensioni per fit
            canvas_width = self.image_canvas.winfo_width()
            canvas_height = self.image_canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                canvas_width = 400
                canvas_height = 400
            
            # Calcola scaling
            img_ratio = img.width / img.height
            canvas_ratio = canvas_width / canvas_height
            
            if img_ratio > canvas_ratio:
                display_width = canvas_width
                display_height = int(canvas_width / img_ratio)
            else:
                display_height = canvas_height
                display_width = int(canvas_height * img_ratio)
            
            # Resize
            img_display = img.resize((display_width, display_height), Image.LANCZOS)
            
            # Converti per tkinter
            from PIL import ImageTk
            self.tk_image = ImageTk.PhotoImage(img_display)
            
            # Pulisci e mostra
            self.image_canvas.delete("all")
            x = (canvas_width - display_width) // 2
            y = (canvas_height - display_height) // 2
            self.image_canvas.create_image(x, y, anchor="nw", image=self.tk_image)
            
            # Aggiorna info
            info = f"{os.path.basename(image_path)} | {img.width}×{img.height} pixels | {img.mode}"
            self.image_info.config(text=info)
            
        except Exception as e:
            self.log(f"❌ Errore visualizzazione immagine: {e}")
    
    def recognize_image(self):
        """Esegue il riconoscimento sull'immagine corrente"""
        if not self.current_image_path or not self.recognizer:
            return
        
        try:
            self.log(f"🔍 Analisi: {os.path.basename(self.current_image_path)}")
            
            # Esegui predizione
            self.current_result = self.recognizer.predict_single_image(self.current_image_path)
            
            if 'error' in self.current_result:
                self.log(f"❌ Errore: {self.current_result['error']}")
                messagebox.showerror("Errore", self.current_result['error'])
                return
            
            # Aggiorna risultati
            self.display_results(self.current_result)
            
            # Abilita esportazione
            self.btn_esporta.config(state="normal")
            
            self.log(f"✅ Riconosciuta: '{self.current_result['predicted_letter']}' "
                    f"({self.current_result['confidence']:.1%})")
            
        except Exception as e:
            self.log(f"❌ Errore riconoscimento: {e}")
            messagebox.showerror("Errore", f"Errore durante il riconoscimento:\n{str(e)}")
    
    def process_batch_images(self, image_paths):
        """Processa multiple immagini"""
        if not self.recognizer:
            return
        
        try:
            self.log(f"📦 Avvio processing batch: {len(image_paths)} immagini")
            
            # Disabilita controlli durante processing
            self.disable_controls(True)
            
            # Predici tutte le immagini
            self.batch_results = self.recognizer.predict_batch(image_paths)
            
            # Mostra prima immagine
            successful = [r for r in self.batch_results if 'error' not in r['result']]
            if successful:
                first_result = successful[0]
                self.current_image_path = first_result['image_path']
                self.current_result = first_result['result']
                self.display_image(first_result['image_path'])
                self.display_results(first_result['result'])
                self.btn_esporta.config(state="normal")
            
            # Riabilita controlli
            self.disable_controls(False)
            
            # Mostra riepilogo
            successful_count = len(successful)
            failed_count = len(image_paths) - successful_count
            
            messagebox.showinfo("Completato",
                              f"Elaborazione completata!\n\n"
                              f"✅ Riconosciute: {successful_count}\n"
                              f"❌ Fallite: {failed_count}\n\n"
                              f"I risultati possono essere esportati.")
            
            self.log(f"📊 Batch completato: {successful_count} successi, {failed_count} falliti")
            
        except Exception as e:
            self.disable_controls(False)
            self.log(f"❌ Errore processing batch: {e}")
            messagebox.showerror("Errore", f"Errore durante l'elaborazione:\n{str(e)}")
    
    def test_rotations(self):
        """Testa robustezza a rotazioni"""
        if not self.current_image_path:
            messagebox.showwarning("Attenzione", "Carica prima un'immagine!")
            return
        
        try:
            self.log(f"🧪 Test rotazioni su: {os.path.basename(self.current_image_path)}")
            
            results = self.recognizer.test_rotation_robustness(self.current_image_path)
            
            # Crea finestra risultati
            result_window = tk.Toplevel(self.root)
            result_window.title("Test Rotazioni")
            result_window.geometry("600x400")
            
            # Treeview per risultati
            tree = ttk.Treeview(result_window, columns=('Angle', 'Letter', 'Confidence', 'Top 3'), 
                              show='headings', height=15)
            
            tree.heading('Angle', text='Angolo (°)')
            tree.heading('Letter', text='Lettera')
            tree.heading('Confidence', text='Confidenza')
            tree.heading('Top 3', text='Top 3')
            
            tree.column('Angle', width=80, anchor='center')
            tree.column('Letter', width=80, anchor='center')
            tree.column('Confidence', width=100, anchor='center')
            tree.column('Top 3', width=200)
            
            for result in results:
                top_3 = ', '.join([f"{p['letter']}({p['confidence']:.0%})" 
                                  for p in result['top_3']])
                tree.insert('', 'end', values=(
                    result['angle'],
                    result['letter'],
                    f"{result['confidence']:.2%}",
                    top_3
                ))
            
            tree.pack(fill="both", expand=True, padx=10, pady=10)
            
            self.log("✅ Test rotazioni completato")
            
        except Exception as e:
            self.log(f"❌ Errore test rotazioni: {e}")
    
    def export_results(self):
        """Esporta risultati"""
        if not self.batch_results and not self.current_result:
            messagebox.showwarning("Attenzione", "Nessun risultato da esportare!")
            return
        
        try:
            # Chiedi formato
            format_window = tk.Toplevel(self.root)
            format_window.title("Formato Esportazione")
            format_window.geometry("300x150")
            
            ttk.Label(format_window, text="Seleziona formato:").pack(pady=10)
            
            format_var = tk.StringVar(value="json")
            ttk.Radiobutton(format_window, text="JSON", variable=format_var, 
                           value="json").pack(pady=5)
            ttk.Radiobutton(format_window, text="CSV", variable=format_var, 
                           value="csv").pack(pady=5)
            
            def do_export():
                if self.batch_results:
                    results_to_export = self.batch_results
                else:
                    results_to_export = [{
                        'image_path': self.current_image_path,
                        'result': self.current_result
                    }]
                
                output_file = self.recognizer.export_results(results_to_export, 
                                                           format_var.get())
                if output_file:
                    self.log(f"💾 Risultati esportati in: {output_file}")
                    messagebox.showinfo("Successo", 
                                      f"Risultati esportati in:\n{output_file}")
                
                format_window.destroy()
            
            ttk.Button(format_window, text="Esporta", 
                      command=do_export).pack(pady=10)
            
        except Exception as e:
            self.log(f"❌ Errore esportazione: {e}")
            messagebox.showerror("Errore", f"Errore durante l'esportazione:\n{str(e)}")
    
    def show_stats(self):
        """Mostra statistiche"""
        if not self.recognizer:
            messagebox.showwarning("Attenzione", "Modello non caricato!")
            return
        
        stats = self.recognizer.stats
        
        # Crea finestra statistiche
        stats_window = tk.Toplevel(self.root)
        stats_window.title("Statistiche")
        stats_window.geometry("500x400")
        
        # Treeview per statistiche
        tree = ttk.Treeview(stats_window, columns=('Metric', 'Value'), 
                          show='headings', height=15)
        
        tree.heading('Metric', text='Metrica')
        tree.heading('Value', text='Valore')
        
        tree.column('Metric', width=200)
        tree.column('Value', width=200)
        
        # Aggiungi statistiche generali
        general_stats = [
            ('Predizioni totali', stats['total_predictions']),
            ('Successi', stats['successful_predictions']),
            ('Falliti', stats['failed_predictions']),
            ('Tasso successo', f"{stats['successful_predictions']/stats['total_predictions']*100:.1f}%" 
             if stats['total_predictions'] > 0 else "N/A"),
            ('Confidenza media', f"{stats['avg_confidence']:.2%}"),
            ('Tempo totale', f"{stats['total_time']:.2f}s"),
            ('Tempo medio/pred', f"{stats['total_time']/stats['total_predictions']:.2f}s" 
             if stats['total_predictions'] > 0 else "N/A")
        ]
        
        for metric, value in general_stats:
            tree.insert('', 'end', values=(metric, value))
        
        tree.pack(fill="both", expand=True, padx=10, pady=10)
    
    def display_results(self, result):
        """Visualizza i risultati"""
        # Lettera principale
        self.letter_label.config(text=result['predicted_letter'])
        
        # Confidenza
        conf_text = f"Confidenza: {result['confidence']:.2%}"
        if 'consensus_votes' in result:
            conf_text += f" ({result['consensus_votes']} voti)"
        if 'confusion_corrected' in result and result['confusion_corrected']:
            conf_text += f"\nCorretto da: {result['original_letter']}"
        
        self.confidence_label.config(text=conf_text)
        
        # Top predictions
        self.top_pred_tree.delete(*self.top_pred_tree.get_children())
        for i, pred in enumerate(result['top_predictions'], 1):
            self.top_pred_tree.insert('', 'end', values=(
                i,
                pred['letter'],
                f"{pred['confidence']:.2%}"
            ))
        
        # Dettagli
        details = []
        details.append(f"File: {os.path.basename(self.current_image_path)}")
        
        if 'original_size' in result:
            details.append(f"Dimensione: {result['original_size'][0]}×{result['original_size'][1]}")
        
        if 'all_orientations' in result:
            details.append(f"\nOrientamenti testati: {len(result['all_orientations'])}")
            for orient in result['all_orientations'][:3]:  # Mostra primi 3
                details.append(f"  {orient['angle']}°: {orient['letter']} ({orient['confidence']:.1%})")
        
        details.append(f"\nTimestamp: {result['timestamp'][:19]}")
        
        self.details_text.delete(1.0, tk.END)
        self.details_text.insert(1.0, '\n'.join(details))
    
    def clear_results(self):
        """Pulisce i risultati"""
        self.letter_label.config(text="?")
        self.confidence_label.config(text="Confidenza: --%")
        self.top_pred_tree.delete(*self.top_pred_tree.get_children())
        self.details_text.delete(1.0, tk.END)
    
    def disable_controls(self, disable=True):
        """Abilita/disabilita controlli"""
        state = "disabled" if disable else "normal"
        self.btn_carica.config(state=state)
        self.btn_carica_cartella.config(state=state)
        self.btn_riconosci.config(state=state)
        self.btn_test.config(state=state)
        self.btn_esporta.config(state=state)
        self.btn_statistiche.config(state=state)
    
    def log(self, message):
        """Aggiunge messaggio al log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
        # Stampa anche su console se verbose
        if config.VERBOSE_LEVEL >= 1:
            print(f"[{timestamp}] {message}")
    
    def clear_log(self):
        """Pulisce il log"""
        self.log_text.delete(1.0, tk.END)
    
    def save_log(self):
        """Salva il log su file"""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("File di testo", "*.txt"), ("Tutti i file", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(self.log_text.get(1.0, tk.END))
                self.log(f"💾 Log salvato in: {file_path}")
            except Exception as e:
                self.log(f"❌ Errore salvataggio log: {e}")

# ============================================
# FUNZIONE PER USO DA TERMINALE
# ============================================
def recognize_from_command_line():
    """
    Funzione per riconoscimento da riga di comando
    Uso: python test_model.py --image percorso/immagine.jpg
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='Test modello riconoscimento lettere')
    parser.add_argument('--image', type=str, help='Percorso immagine da riconoscere')
    parser.add_argument('--folder', type=str, help='Cartella con immagini da riconoscere')
    parser.add_argument('--batch', action='store_true', help='Modalità batch')
    parser.add_argument('--output', type=str, help='File di output per risultati')
    parser.add_argument('--test-rotations', action='store_true', help='Testa tutti gli orientamenti')
    parser.add_argument('--no-gui', action='store_true', help='Senza interfaccia grafica')
    parser.add_argument('--model', type=str, help='Percorso modello personalizzato')
    parser.add_argument('--best-model', action='store_true', help='Usa il best model')
    parser.add_argument('--verbose', type=int, default=2, help='Livello verbose (0-3)')
    
    args = parser.parse_args()
    
    # Configura verbose level
    config.VERBOSE_LEVEL = args.verbose
    
    # Inizializza riconoscitore
    try:
        recognizer = AdvancedLetterRecognizer(
            model_path=args.model,
            use_best_model=args.best_model
        )
    except Exception as e:
        print(f"❌ Errore inizializzazione: {e}")
        return
    
    if args.image:
        # Singola immagine
        print(f"\n🔍 Analisi immagine: {args.image}")
        
        if args.test_rotations:
            print("\n🧪 Test tutti gli orientamenti:")
            results = recognizer.test_rotation_robustness(args.image)
            print("\n📊 Risultati:")
            for result in results:
                print(f"  {result['angle']:3d}°: {result['letter']} ({result['confidence']:.2%})")
        else:
            result = recognizer.predict_single_image(args.image)
            
            if 'error' in result:
                print(f"❌ Errore: {result['error']}")
            else:
                print(f"\n✅ RISULTATO:")
                print(f"   Lettera: {result['predicted_letter']}")
                print(f"   Confidenza: {result['confidence']:.2%}")
                
                if 'consensus_votes' in result:
                    print(f"   Consenso: {result['consensus_votes']} voti")
                
                if 'confusion_corrected' in result and result['confusion_corrected']:
                    print(f"   Correzione: {result['original_letter']} → {result['predicted_letter']}")
                
                print(f"\n📊 TOP {config.SHOW_TOP_PREDICTIONS}:")
                for i, pred in enumerate(result['top_predictions'], 1):
                    print(f"   {i}. {pred['letter']}: {pred['confidence']:.2%}")
        
        # Salva risultato se richiesto
        if args.output:
            results_to_save = [{'image_path': args.image, 'result': result}]
            output_file = recognizer.export_results(results_to_save, 
                                                  'json' if args.output.endswith('.json') else 'csv')
            print(f"\n💾 Risultato salvato in: {output_file}")
    
    elif args.folder:
        # Cartella di immagini
        print(f"\n📂 Analisi cartella: {args.folder}")
        
        # Trova tutte le immagini
        extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']
        image_files = []
        
        for ext in extensions:
            image_files.extend(Path(args.folder).glob(f'*{ext}'))
            image_files.extend(Path(args.folder).glob(f'*{ext.upper()}'))
        
        image_paths = [str(f) for f in image_files]
        
        if not image_paths:
            print("❌ Nessuna immagine trovata nella cartella")
            return
        
        print(f"Trovate {len(image_paths)} immagini")
        
        # Processa tutte le immagini
        results = recognizer.predict_batch(image_paths)
        
        # Salva risultati se richiesto
        if args.output:
            output_file = recognizer.export_results(results, 
                                                  'json' if args.output.endswith('.json') else 'csv')
            print(f"\n💾 Risultati salvati in: {output_file}")
    
    else:
        # Nessun argomento, avvia GUI (se non disabilitata)
        if not args.no_gui:
            print("Avvio interfaccia grafica...")
            start_gui()
        else:
            print("\nℹ️  Modalità da terminale")
            print("Usa --help per vedere le opzioni disponibili")

# ============================================
# AVVIO INTERFACCIA GRAFICA
# ============================================
def start_gui():
    """Avvia l'interfaccia grafica"""
    root = tk.Tk()
    app = LetterRecognitionGUI(root)
    
    # Centra la finestra
    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = (root.winfo_screenwidth() // 2) - (width // 2)
    y = (root.winfo_screenheight() // 2) - (height // 2)
    root.geometry(f'{width}x{height}+{x}+{y}')
    
    root.mainloop()

# ============================================
# MAIN
# ============================================
if __name__ == "__main__":
    print("\n" + "="*60)
    print("🧪 TEST MODELLO RICONOSCIMENTO LETTERE ROTATE")
    print("="*60)
    
    # Verifica se ci sono argomenti da riga di comando
    if len(sys.argv) > 1:
        recognize_from_command_line()
    else:
        # Avvia GUI di default
        start_gui()