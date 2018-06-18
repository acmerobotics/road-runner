import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent
import com.intellij.openapi.ui.Messages

class HelloAction : AnAction("Hello") {
    override fun actionPerformed(event: AnActionEvent) {
        val project = event.getProject()
        Messages.showMessageDialog(project, "Hello world!", "Greeting", Messages.getInformationIcon())
    }
}