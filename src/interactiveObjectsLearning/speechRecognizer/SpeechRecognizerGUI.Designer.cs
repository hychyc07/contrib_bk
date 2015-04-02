namespace RobotSpeech
{
    partial class SpeechRecognizerGUI
    {
        /// <summary>
        /// Variable nécessaire au concepteur.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Nettoyage des ressources utilisées.
        /// </summary>
        /// <param name="disposing">true si les ressources managées doivent être supprimées ; sinon, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Code généré par le Concepteur Windows Form

        /// <summary>
        /// Méthode requise pour la prise en charge du concepteur - ne modifiez pas
        /// le contenu de cette méthode avec l'éditeur de code.
        /// </summary>
        private void InitializeComponent()
        {
            this.groupBoxReco = new System.Windows.Forms.GroupBox();
            this.label1 = new System.Windows.Forms.Label();
            this.listBoxLanguage = new System.Windows.Forms.ListBox();
            this.groupBoxTTS = new System.Windows.Forms.GroupBox();
            this.buttonSay = new System.Windows.Forms.Button();
            this.textBoxSay = new System.Windows.Forms.TextBox();
            this.groupBoxReco.SuspendLayout();
            this.groupBoxTTS.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBoxReco
            // 
            this.groupBoxReco.Controls.Add(this.label1);
            this.groupBoxReco.Controls.Add(this.listBoxLanguage);
            this.groupBoxReco.Location = new System.Drawing.Point(12, 12);
            this.groupBoxReco.Name = "groupBoxReco";
            this.groupBoxReco.Size = new System.Drawing.Size(245, 112);
            this.groupBoxReco.TabIndex = 6;
            this.groupBoxReco.TabStop = false;
            this.groupBoxReco.Text = "Speech Recognition";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 24);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(90, 13);
            this.label1.TabIndex = 9;
            this.label1.Text = "Choose language";
            // 
            // listBoxLanguage
            // 
            this.listBoxLanguage.FormattingEnabled = true;
            this.listBoxLanguage.Location = new System.Drawing.Point(6, 40);
            this.listBoxLanguage.Name = "listBoxLanguage";
            this.listBoxLanguage.Size = new System.Drawing.Size(233, 56);
            this.listBoxLanguage.TabIndex = 8;
            this.listBoxLanguage.SelectedIndexChanged += new System.EventHandler(this.listBoxLanguage_SelectedIndexChanged);
            // 
            // groupBoxTTS
            // 
            this.groupBoxTTS.Controls.Add(this.buttonSay);
            this.groupBoxTTS.Controls.Add(this.textBoxSay);
            this.groupBoxTTS.Location = new System.Drawing.Point(263, 12);
            this.groupBoxTTS.Name = "groupBoxTTS";
            this.groupBoxTTS.Size = new System.Drawing.Size(200, 112);
            this.groupBoxTTS.TabIndex = 7;
            this.groupBoxTTS.TabStop = false;
            this.groupBoxTTS.Text = "Text To Speech";
            // 
            // buttonSay
            // 
            this.buttonSay.Location = new System.Drawing.Point(119, 50);
            this.buttonSay.Name = "buttonSay";
            this.buttonSay.Size = new System.Drawing.Size(75, 23);
            this.buttonSay.TabIndex = 3;
            this.buttonSay.Text = "Say it !";
            this.buttonSay.UseVisualStyleBackColor = true;
            this.buttonSay.Click += new System.EventHandler(this.buttonSay_Click);
            // 
            // textBoxSay
            // 
            this.textBoxSay.Location = new System.Drawing.Point(6, 24);
            this.textBoxSay.Name = "textBoxSay";
            this.textBoxSay.Size = new System.Drawing.Size(188, 20);
            this.textBoxSay.TabIndex = 2;
            this.textBoxSay.Text = "Let say something intelligent";
            // 
            // speechRecognizerGUI
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(478, 146);
            this.Controls.Add(this.groupBoxTTS);
            this.Controls.Add(this.groupBoxReco);
            this.Name = "speechRecognizerGUI";
            this.Text = "Speech Manager";
            this.groupBoxReco.ResumeLayout(false);
            this.groupBoxReco.PerformLayout();
            this.groupBoxTTS.ResumeLayout(false);
            this.groupBoxTTS.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBoxReco;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ListBox listBoxLanguage;
        private System.Windows.Forms.GroupBox groupBoxTTS;
        private System.Windows.Forms.Button buttonSay;
        private System.Windows.Forms.TextBox textBoxSay;

    }
}

