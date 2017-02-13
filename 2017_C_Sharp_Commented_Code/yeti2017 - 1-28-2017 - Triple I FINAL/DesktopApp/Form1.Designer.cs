namespace Yeti2015.DesktopApp
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.dataGridViewDebug = new System.Windows.Forms.DataGridView();
            this.yetiX = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.yetiY = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.yetiHeading = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.left = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.right = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.turn = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.lSpeed = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.rSpeed = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.landmarkCount = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.pError = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.buttonDisable = new System.Windows.Forms.Button();
            this.buttonEnableManual = new System.Windows.Forms.Button();
            this.buttonEnableAuto = new System.Windows.Forms.Button();
            this.checkBoxUpdateList = new System.Windows.Forms.CheckBox();
            this.checkBoxUpdateGUI = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.tableLayoutPanel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.dataGridViewDebug)).BeginInit();
            this.flowLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // pictureBox1
            // 
            this.pictureBox1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pictureBox1.Location = new System.Drawing.Point(3, 3);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(594, 594);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.pictureBox1.TabIndex = 0;
            this.pictureBox1.TabStop = false;
            this.pictureBox1.Click += new System.EventHandler(this.pictureBox1_Click);
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 600F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.Controls.Add(this.pictureBox1, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.dataGridViewDebug, 1, 0);
            this.tableLayoutPanel1.Controls.Add(this.flowLayoutPanel1, 1, 1);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 600F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1280, 688);
            this.tableLayoutPanel1.TabIndex = 1;
            // 
            // dataGridViewDebug
            // 
            this.dataGridViewDebug.AllowUserToAddRows = false;
            this.dataGridViewDebug.AllowUserToDeleteRows = false;
            this.dataGridViewDebug.AutoSizeColumnsMode = System.Windows.Forms.DataGridViewAutoSizeColumnsMode.AllCells;
            this.dataGridViewDebug.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dataGridViewDebug.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.yetiX,
            this.yetiY,
            this.yetiHeading,
            this.left,
            this.right,
            this.turn,
            this.lSpeed,
            this.rSpeed,
            this.landmarkCount,
            this.pError});
            this.dataGridViewDebug.Dock = System.Windows.Forms.DockStyle.Fill;
            this.dataGridViewDebug.Location = new System.Drawing.Point(603, 3);
            this.dataGridViewDebug.Name = "dataGridViewDebug";
            this.dataGridViewDebug.ReadOnly = true;
            this.dataGridViewDebug.Size = new System.Drawing.Size(674, 594);
            this.dataGridViewDebug.TabIndex = 1;
            // 
            // yetiX
            // 
            this.yetiX.HeaderText = "Yeti X";
            this.yetiX.Name = "yetiX";
            this.yetiX.ReadOnly = true;
            this.yetiX.Width = 50;
            // 
            // yetiY
            // 
            this.yetiY.HeaderText = "Yeti Y";
            this.yetiY.Name = "yetiY";
            this.yetiY.ReadOnly = true;
            this.yetiY.Width = 50;
            // 
            // yetiHeading
            // 
            this.yetiHeading.HeaderText = "Yeti Heading";
            this.yetiHeading.Name = "yetiHeading";
            this.yetiHeading.ReadOnly = true;
            this.yetiHeading.Width = 86;
            // 
            // left
            // 
            this.left.HeaderText = "Left";
            this.left.Name = "left";
            this.left.ReadOnly = true;
            this.left.Width = 50;
            // 
            // right
            // 
            this.right.HeaderText = "Right";
            this.right.Name = "right";
            this.right.ReadOnly = true;
            this.right.Width = 57;
            // 
            // turn
            // 
            this.turn.HeaderText = "Turn";
            this.turn.Name = "turn";
            this.turn.ReadOnly = true;
            this.turn.Width = 54;
            // 
            // lSpeed
            // 
            this.lSpeed.HeaderText = "LSpeed";
            this.lSpeed.Name = "lSpeed";
            this.lSpeed.ReadOnly = true;
            this.lSpeed.Width = 69;
            // 
            // rSpeed
            // 
            this.rSpeed.HeaderText = "RSpeed";
            this.rSpeed.Name = "rSpeed";
            this.rSpeed.ReadOnly = true;
            this.rSpeed.Width = 71;
            // 
            // landmarkCount
            // 
            this.landmarkCount.HeaderText = "Landmark Count";
            this.landmarkCount.Name = "landmarkCount";
            this.landmarkCount.ReadOnly = true;
            this.landmarkCount.Width = 101;
            // 
            // pError
            // 
            this.pError.HeaderText = "P Error";
            this.pError.Name = "pError";
            this.pError.ReadOnly = true;
            this.pError.Width = 60;
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.buttonDisable);
            this.flowLayoutPanel1.Controls.Add(this.buttonEnableManual);
            this.flowLayoutPanel1.Controls.Add(this.buttonEnableAuto);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxUpdateList);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxUpdateGUI);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.FlowDirection = System.Windows.Forms.FlowDirection.RightToLeft;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(603, 603);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(674, 123);
            this.flowLayoutPanel1.TabIndex = 2;
            // 
            // buttonDisable
            // 
            this.buttonDisable.Location = new System.Drawing.Point(596, 3);
            this.buttonDisable.Name = "buttonDisable";
            this.buttonDisable.Size = new System.Drawing.Size(75, 23);
            this.buttonDisable.TabIndex = 2;
            this.buttonDisable.Text = "Disable";
            this.buttonDisable.UseVisualStyleBackColor = true;
            // 
            // buttonEnableManual
            // 
            this.buttonEnableManual.AutoSize = true;
            this.buttonEnableManual.Location = new System.Drawing.Point(421, 3);
            this.buttonEnableManual.Name = "buttonEnableManual";
            this.buttonEnableManual.Size = new System.Drawing.Size(169, 30);
            this.buttonEnableManual.TabIndex = 0;
            this.buttonEnableManual.Text = "Enable Manual Mode";
            this.buttonEnableManual.UseVisualStyleBackColor = true;
            // 
            // buttonEnableAuto
            // 
            this.buttonEnableAuto.AutoSize = true;
            this.buttonEnableAuto.Location = new System.Drawing.Point(207, 3);
            this.buttonEnableAuto.Name = "buttonEnableAuto";
            this.buttonEnableAuto.Size = new System.Drawing.Size(208, 30);
            this.buttonEnableAuto.TabIndex = 1;
            this.buttonEnableAuto.Text = "Enable Autonomous Mode";
            this.buttonEnableAuto.UseVisualStyleBackColor = true;
            // 
            // checkBoxUpdateList
            // 
            this.checkBoxUpdateList.AutoSize = true;
            this.checkBoxUpdateList.Location = new System.Drawing.Point(121, 3);
            this.checkBoxUpdateList.Name = "checkBoxUpdateList";
            this.checkBoxUpdateList.Size = new System.Drawing.Size(80, 17);
            this.checkBoxUpdateList.TabIndex = 3;
            this.checkBoxUpdateList.Text = "Update List";
            this.checkBoxUpdateList.UseVisualStyleBackColor = true;
            this.checkBoxUpdateList.CheckedChanged += new System.EventHandler(this.checkBoxUpdateList_CheckedChanged);
            // 
            // checkBoxUpdateGUI
            // 
            this.checkBoxUpdateGUI.AutoSize = true;
            this.checkBoxUpdateGUI.Checked = true;
            this.checkBoxUpdateGUI.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxUpdateGUI.Location = new System.Drawing.Point(32, 3);
            this.checkBoxUpdateGUI.Name = "checkBoxUpdateGUI";
            this.checkBoxUpdateGUI.Size = new System.Drawing.Size(83, 17);
            this.checkBoxUpdateGUI.TabIndex = 4;
            this.checkBoxUpdateGUI.Text = "Update GUI";
            this.checkBoxUpdateGUI.UseVisualStyleBackColor = true;
            this.checkBoxUpdateGUI.CheckedChanged += new System.EventHandler(this.checkBoxUpdateGUI_CheckedChanged);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1280, 688);
            this.Controls.Add(this.tableLayoutPanel1);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Form1";
            this.Text = "Yeti 2017";
            this.WindowState = System.Windows.Forms.FormWindowState.Maximized;
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.Form1_FormClosed);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.tableLayoutPanel1.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.dataGridViewDebug)).EndInit();
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.DataGridView dataGridViewDebug;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Button buttonDisable;
        private System.Windows.Forms.Button buttonEnableManual;
        private System.Windows.Forms.Button buttonEnableAuto;
        private System.Windows.Forms.DataGridViewTextBoxColumn yetiX;
        private System.Windows.Forms.DataGridViewTextBoxColumn yetiY;
        private System.Windows.Forms.DataGridViewTextBoxColumn yetiHeading;
        private System.Windows.Forms.DataGridViewTextBoxColumn left;
        private System.Windows.Forms.DataGridViewTextBoxColumn right;
        private System.Windows.Forms.DataGridViewTextBoxColumn turn;
        private System.Windows.Forms.DataGridViewTextBoxColumn lSpeed;
        private System.Windows.Forms.DataGridViewTextBoxColumn rSpeed;
        private System.Windows.Forms.DataGridViewTextBoxColumn landmarkCount;
        private System.Windows.Forms.DataGridViewTextBoxColumn pError;
        private System.Windows.Forms.CheckBox checkBoxUpdateList;
        private System.Windows.Forms.CheckBox checkBoxUpdateGUI;
    }
}

