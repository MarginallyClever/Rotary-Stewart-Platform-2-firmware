package RotaryStewartPlatform2;

import java.awt.FlowLayout;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.EmptyBorder;

public class JLabelledTextField extends JPanel {
	/**
	 * 
	 */
	private static final long serialVersionUID = -6112042391576602225L;
	private JLabel label;
	private JTextField field;
	
	JLabelledTextField(String value_text,String label_text) {
		label = new JLabel(label_text);
		field = new JTextField(value_text);

		JPanel margin = new JPanel();
        this.add(margin);
        margin.setBorder(new EmptyBorder(5, 5, 5, 5));
		JPanel container = new JPanel();
		container.setLayout(new FlowLayout());
		margin.add(container);
		container.add(label);
		container.add(field);
	}
}
